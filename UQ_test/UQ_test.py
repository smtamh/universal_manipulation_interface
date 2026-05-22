import argparse
import csv
import json
import math
import os
import pathlib
import sys

import numpy as np
import torch


THIS_DIR = pathlib.Path(__file__).resolve().parent
BASELINES_DIR = THIS_DIR.parent / 'UQ_baselines'
for path in [
    BASELINES_DIR,
    BASELINES_DIR / 'logpZO',
    BASELINES_DIR / 'RND',
    BASELINES_DIR / 'CFM',
    BASELINES_DIR / 'NatPN',
    BASELINES_DIR / 'PCA_kmeans',
]:
    sys.path.append(str(path))


DEFAULT_RISK = {
    'RND': 'high',
    'CFM': 'high',
    'logpZO': 'high',
    'NatPN': 'low',
    'PCA_kmeans': 'high',
}


def adjust_xshape(x, in_dim):
    total_dim = x.shape[1]
    remain_dim = total_dim % in_dim
    if remain_dim > 0:
        pad = in_dim - remain_dim
        x = torch.cat([x, torch.zeros(x.shape[0], pad, device=x.device)], dim=1)
        total_dim += pad
    reshaped_dim = total_dim // in_dim
    if reshaped_dim % 4 != 0:
        extra_pad = (4 - (reshaped_dim % 4)) * in_dim
        x = torch.cat([x, torch.zeros(x.shape[0], extra_pad, device=x.device)], dim=1)
    return x.reshape(x.shape[0], -1, in_dim)


def load_labels(labels_path):
    labels = dict()
    with open(labels_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            episode_id = int(row['episode_id'])
            success = int(row['success'])
            split = row.get('split', '').strip()
            labels[episode_id] = {
                'success': success,
                'split': split,
            }
    return labels


def select_episode_ids(uq_log_dir, labels, split=None):
    episode_ids = []
    for path in sorted(pathlib.Path(uq_log_dir).glob('*.pt'), key=lambda p: int(p.stem)):
        episode_id = int(path.stem)
        if episode_id not in labels:
            continue
        if split is not None and labels[episode_id]['split'] != split:
            continue
        episode_ids.append(episode_id)
    return episode_ids


def load_episode_records(uq_log_dir, episode_id):
    path = pathlib.Path(uq_log_dir) / f'{episode_id}.pt'
    data = torch.load(path)
    return data['records']


def records_to_tensors(records, device):
    global_cond = torch.from_numpy(
        np.stack([record['global_cond'] for record in records], axis=0)
    ).to(device=device, dtype=torch.float32)
    normalized_action_pred = torch.from_numpy(
        np.stack([record['normalized_action_pred'] for record in records], axis=0)
    ).to(device=device, dtype=torch.float32)
    action_pred = torch.from_numpy(
        np.stack([record['action_pred'] for record in records], axis=0)
    ).to(device=device, dtype=torch.float32)
    timestamps = np.asarray([record['obs_timestamp'] for record in records], dtype=np.float64)
    return {
        'global_cond': global_cond,
        'normalized_action_pred': normalized_action_pred,
        'action_pred': action_pred,
        'timestamps': timestamps,
    }


def load_baseline_model(baseline_name, sample_tensors, ckpt_path=None, device='cpu'):
    if ckpt_path is None:
        ckpt_path = BASELINES_DIR / baseline_name / f'umi_{baseline_name}.ckpt'
    ckpt = torch.load(ckpt_path, map_location=device)
    global_cond_dim = sample_tensors['global_cond'].shape[1]
    action_dim = sample_tensors['normalized_action_pred'].shape[-1]

    if baseline_name == 'RND':
        from net import RNDPolicy
        net = RNDPolicy(action_dim, global_cond_dim)
    elif baseline_name == 'CFM':
        import net_CFM as Net
        net = Net.get_unet(action_dim)
    elif baseline_name == 'logpZO':
        import CFM.net_CFM as Net
        net = Net.get_unet(action_dim)
        net.global_eps = None
    elif baseline_name == 'NatPN':
        from net_natpn import get_net
        estimator = get_net()
        net = estimator._init_model(
            output_type='categorical',
            input_size=torch.Size([global_cond_dim]),
            num_classes=64
        )
    elif baseline_name == 'PCA_kmeans':
        from net_PCA import PCAKMeansNet
        pca_components = ckpt['model']['pca_components']
        centroids = ckpt['model']['centroids']
        emb_dim = pca_components.shape[0]
        n_clusters = centroids.shape[0]
        net = PCAKMeansNet(input_dim=global_cond_dim, emb_dim=emb_dim, n_clusters=n_clusters)
    else:
        raise ValueError(f'Unsupported baseline: {baseline_name}')

    net.load_state_dict(ckpt['model'])
    net = net.to(device)
    net.eval()
    return net


def compute_scores(baseline_name, baseline_model, episode_tensors):
    global_cond = episode_tensors['global_cond']
    normalized_action_pred = episode_tensors['normalized_action_pred']
    action_dim = normalized_action_pred.shape[-1]

    if baseline_name == 'RND':
        with torch.no_grad():
            return baseline_model(normalized_action_pred, global_cond).sum(dim=1).detach().cpu().numpy()

    if baseline_name == 'CFM':
        observation = adjust_xshape(global_cond, action_dim)
        nstep = 5
        with torch.no_grad():
            timesteps = torch.linspace(1, 0, nstep + 1, device=observation.device)[:-1]
            timesteps = (timesteps * 100).long()
            predicted_v = []
            xnow = observation
            for t in timesteps:
                pred_v = baseline_model(xnow, t)
                predicted_v.append(pred_v.detach().cpu().numpy())
                xnow = xnow - pred_v / nstep
        predicted_v = np.asarray(predicted_v).reshape(nstep, len(global_cond), -1)
        return np.std(predicted_v, axis=0).mean(axis=1)

    if baseline_name == 'logpZO':
        observation = adjust_xshape(global_cond, action_dim)
        with torch.no_grad():
            timesteps = torch.zeros(observation.shape[0], device=observation.device)
            pred_v = baseline_model(observation, timesteps)
            observation = observation + pred_v
            logpzo = observation.reshape(len(observation), -1).pow(2).sum(dim=-1)
        return logpzo.detach().cpu().numpy()

    if baseline_name == 'NatPN':
        with torch.no_grad():
            _, log_prob = baseline_model.forward(global_cond)
        return log_prob.detach().cpu().numpy()

    if baseline_name == 'PCA_kmeans':
        with torch.no_grad():
            scores = baseline_model(global_cond)
        return scores.detach().cpu().numpy()

    raise ValueError(f'Unsupported baseline: {baseline_name}')


def align_threshold(threshold, length):
    threshold = np.asarray(threshold, dtype=np.float32)
    if len(threshold) >= length:
        return threshold[:length]
    pad = np.repeat(threshold[-1], length - len(threshold))
    return np.concatenate([threshold, pad], axis=0)


def detect_crossing(scores, threshold, risk):
    if risk == 'high':
        mask = scores >= threshold
    else:
        mask = scores <= threshold
    if np.any(mask):
        first_idx = int(np.nonzero(mask)[0][0])
        return 1, first_idx
    return 0, None


def compute_metrics(results):
    labeled = [result for result in results if result['success'] is not None]
    if not labeled:
        return None
    y_true = np.asarray([1 - int(result['success']) for result in labeled], dtype=np.int64)
    y_pred = np.asarray([int(result['positive']) for result in labeled], dtype=np.int64)

    tp = int(np.sum((y_true == 1) & (y_pred == 1)))
    tn = int(np.sum((y_true == 0) & (y_pred == 0)))
    fp = int(np.sum((y_true == 0) & (y_pred == 1)))
    fn = int(np.sum((y_true == 1) & (y_pred == 0)))

    tpr = tp / max(tp + fn, 1)
    tnr = tn / max(tn + fp, 1)
    balanced_accuracy = 0.5 * (tpr + tnr)
    failure_rate = y_true.mean()
    success_rate = 1.0 - failure_rate
    weighted_accuracy = failure_rate * tpr + success_rate * tnr

    detection_steps = [result['first_detection_step'] for result in labeled if result['first_detection_step'] is not None]
    metrics = {
        'tp': tp,
        'tn': tn,
        'fp': fp,
        'fn': fn,
        'tpr': tpr,
        'tnr': tnr,
        'balanced_accuracy': balanced_accuracy,
        'weighted_accuracy': weighted_accuracy,
        'mean_detection_step': float(np.mean(detection_steps)) if detection_steps else None,
    }
    return metrics


def calibrate(args):
    device = torch.device(args.device if torch.cuda.is_available() else 'cpu')
    labels = load_labels(args.labels)
    episode_ids = select_episode_ids(args.uq_log_dir, labels, split=args.split)
    success_ids = [episode_id for episode_id in episode_ids if labels[episode_id]['success'] == 1]
    if not success_ids:
        raise ValueError('No successful calibration episodes found.')

    sample_tensors = records_to_tensors(load_episode_records(args.uq_log_dir, success_ids[0]), device)
    baseline_model = load_baseline_model(
        baseline_name=args.baseline,
        sample_tensors=sample_tensors,
        ckpt_path=args.ckpt,
        device=device
    )

    score_trajs = []
    for episode_id in success_ids:
        episode_tensors = records_to_tensors(load_episode_records(args.uq_log_dir, episode_id), device)
        scores = compute_scores(args.baseline, baseline_model, episode_tensors)
        score_trajs.append(scores)

    min_len = min(len(scores) for scores in score_trajs)
    aligned = np.stack([scores[:min_len] for scores in score_trajs], axis=0)
    risk = args.risk or DEFAULT_RISK[args.baseline]
    quantile = 1.0 - args.alpha if risk == 'high' else args.alpha
    threshold = np.quantile(aligned, quantile, axis=0).astype(np.float32)

    output = {
        'baseline': args.baseline,
        'alpha': args.alpha,
        'risk': risk,
        'threshold': threshold,
        'calibration_episode_ids': success_ids,
        'min_calibration_length': int(min_len),
    }
    pathlib.Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    torch.save(output, args.output)
    print(f'Saved calibration to {args.output}')
    print(f'Baseline: {args.baseline}')
    print(f'Risk direction: {risk}')
    print(f'Calibration episodes: {len(success_ids)}')
    print(f'Threshold length: {len(threshold)}')


def apply_calibration(args):
    device = torch.device(args.device if torch.cuda.is_available() else 'cpu')
    calibration = torch.load(args.calibration)
    baseline = calibration['baseline']
    risk = calibration['risk']
    threshold = calibration['threshold']

    labels = load_labels(args.labels) if args.labels else dict()
    if labels:
        episode_ids = select_episode_ids(args.uq_log_dir, labels, split=args.split)
    else:
        episode_ids = [int(path.stem) for path in sorted(pathlib.Path(args.uq_log_dir).glob('*.pt'), key=lambda p: int(p.stem))]
    if not episode_ids:
        raise ValueError('No episodes selected for scoring.')

    sample_tensors = records_to_tensors(load_episode_records(args.uq_log_dir, episode_ids[0]), device)
    baseline_model = load_baseline_model(
        baseline_name=baseline,
        sample_tensors=sample_tensors,
        ckpt_path=args.ckpt,
        device=device
    )

    results = []
    for episode_id in episode_ids:
        episode_tensors = records_to_tensors(load_episode_records(args.uq_log_dir, episode_id), device)
        scores = compute_scores(baseline, baseline_model, episode_tensors)
        step_threshold = align_threshold(threshold, len(scores))
        positive, first_idx = detect_crossing(scores, step_threshold, risk)
        result = {
            'episode_id': episode_id,
            'success': labels.get(episode_id, {}).get('success'),
            'positive': positive,
            'first_detection_step': first_idx,
            'num_policy_steps': len(scores),
            'scores': scores.tolist(),
            'threshold': step_threshold.tolist(),
        }
        results.append(result)

    metrics = compute_metrics(results)
    output = {
        'baseline': baseline,
        'risk': risk,
        'results': results,
        'metrics': metrics,
    }
    pathlib.Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(output, f, indent=2)
    print(f'Saved scoring results to {args.output}')
    if metrics is not None:
        print(json.dumps(metrics, indent=2))


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)

    calibrate_parser = subparsers.add_parser('calibrate')
    calibrate_parser.add_argument('--baseline', required=True, choices=list(DEFAULT_RISK.keys()))
    calibrate_parser.add_argument('--uq-log-dir', required=True)
    calibrate_parser.add_argument('--labels', required=True)
    calibrate_parser.add_argument('--split', default='calib')
    calibrate_parser.add_argument('--alpha', default=0.1, type=float)
    calibrate_parser.add_argument('--risk', choices=['high', 'low'], default=None)
    calibrate_parser.add_argument('--ckpt', default=None)
    calibrate_parser.add_argument('--device', default='cuda')
    calibrate_parser.add_argument('--output', required=True)
    calibrate_parser.set_defaults(func=calibrate)

    apply_parser = subparsers.add_parser('apply')
    apply_parser.add_argument('--calibration', required=True)
    apply_parser.add_argument('--uq-log-dir', required=True)
    apply_parser.add_argument('--labels', default=None)
    apply_parser.add_argument('--split', default='test')
    apply_parser.add_argument('--ckpt', default=None)
    apply_parser.add_argument('--device', default='cuda')
    apply_parser.add_argument('--output', required=True)
    apply_parser.set_defaults(func=apply_calibration)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
