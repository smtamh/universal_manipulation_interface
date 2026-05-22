import sys
sys.stdout = open(sys.stdout.fileno(), mode='w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode='w', buffering=1)
import torch
import dill
import hydra
from omegaconf import OmegaConf
import pathlib
from diffusion_policy.workspace.base_workspace import BaseWorkspace

OmegaConf.register_new_resolver("eval", eval, replace=True)

from hydra.core.hydra_config import HydraConfig


@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'diffusion_policy', 'config'))
)
def main(cfg: OmegaConf):
    cfg['_target_'] = (
        'diffusion_policy.workspace.train_diffusion_unet_image_workspace_get_data.TrainDiffusionUnetImageWorkspace'
    )

    run_dir = pathlib.Path(HydraConfig.get().runtime.output_dir).expanduser().resolve()
    cfg['checkpoint'] = str(run_dir / 'checkpoints' / 'latest.ckpt')

    if 'output_file' in cfg:
        output_file = str(pathlib.Path(cfg.output_file).expanduser().resolve())
    else:
        output_file = str(run_dir / 'umi_faildetect_train.pt')
    cfg['logging'] = output_file

    cfg['dataloader']['shuffle'] = False
    OmegaConf.resolve(cfg)
    cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = cls(cfg)
    payload = torch.load(open(cfg['checkpoint'], 'rb'), pickle_module=dill)
    workspace.logging = cfg['logging']
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)
    workspace.run()


if __name__ == "__main__":
    main()

'''
  python save_data.py \
    --config-name=train_diffusion_unet_timm_umi_workspace \
    training.seed=42 \
    training.device=cuda:0 \
    task.dataset_path=/home/dyros/smtamh/umi/data/dataset.zarr.zip \
    hydra.run.dir='data/outputs/2026.04.29/train_diffusion_unet_timm_umi'
'''