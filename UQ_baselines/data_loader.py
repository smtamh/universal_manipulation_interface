import pathlib

import torch


def adjust_xshape(x, in_dim):
    total_dim = x.shape[1]
    # Calculate the padding needed to make total_dim a multiple of in_dim
    remain_dim = total_dim % in_dim
    if remain_dim > 0:
        pad = in_dim - remain_dim
        total_dim += pad
        x = torch.cat([x, torch.zeros(x.shape[0], pad, device=x.device)], dim=1)
    # Calculate the padding needed to make (total_dim // in_dim) a multiple of 4
    reshaped_dim = total_dim // in_dim
    if reshaped_dim % 4 != 0:
        extra_pad = (4 - (reshaped_dim % 4)) * in_dim
        x = torch.cat([x, torch.zeros(x.shape[0], extra_pad, device=x.device)], dim=1)
    return x.reshape(x.shape[0], -1, in_dim)


def _find_default_data_file() -> pathlib.Path:
    outputs_dir = pathlib.Path(__file__).resolve().parents[1] / 'data' / 'outputs'
    matches = list(outputs_dir.glob('*/train_diffusion_unet_timm_umi/umi_faildetect_train.pt'))
    if not matches:
        raise FileNotFoundError(
            'Could not find umi_faildetect_train.pt under data/outputs/*/train_diffusion_unet_timm_umi/. '
            'Pass --data-file explicitly.'
        )
    matches.sort(key=lambda p: p.stat().st_mtime)
    return matches[-1]


def get_data(data_file=None, adjust_shape=True, action_horizon=16):
    if data_file is None:
        data_path = _find_default_data_file()
    else:
        data_path = pathlib.Path(data_file).expanduser().resolve()

    data = torch.load(data_path)
    X, Y = data['X'], data['Y']
    in_dim = Y.shape[-1] // action_horizon
    if adjust_shape:
        X = adjust_xshape(X, in_dim)
    return X, Y
