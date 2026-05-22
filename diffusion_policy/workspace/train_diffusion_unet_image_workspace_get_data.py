if __name__ == "__main__":
    import sys
    import os
    import pathlib

    ROOT_DIR = str(pathlib.Path(__file__).parent.parent.parent)
    sys.path.append(ROOT_DIR)
    os.chdir(ROOT_DIR)


import hydra
import torch
from omegaconf import OmegaConf
import pathlib
from torch.utils.data import DataLoader
import copy
import random
import numpy as np
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.diffusion_unet_image_policy import DiffusionUnetImagePolicy
from diffusion_policy.dataset.base_dataset import BaseDataset, BaseImageDataset
from diffusion_policy.common.pytorch_util import dict_apply

OmegaConf.register_new_resolver("eval", eval, replace=True)

class TrainDiffusionUnetImageWorkspace(BaseWorkspace):
    include_keys = ['global_step', 'epoch']
    exclude_keys = tuple()

    def __init__(self, cfg: OmegaConf, output_dir=None):
        super().__init__(cfg, output_dir=output_dir)

        # set seed
        seed = cfg.training.seed
        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        # configure model
        self.model: DiffusionUnetImagePolicy = hydra.utils.instantiate(cfg.policy)
        self.ema_model: DiffusionUnetImagePolicy = None
        if cfg.training.use_ema:
            self.ema_model = copy.deepcopy(self.model)

        obs_encoder_lr = cfg.optimizer.lr
        if cfg.policy.obs_encoder.pretrained:
            obs_encoder_lr *= 0.1
            print("==> reduce pretrained obs_encoder's lr")
        obs_encoder_params = list()
        for param in self.model.obs_encoder.parameters():
            if param.requires_grad:
                obs_encoder_params.append(param)
        print(f'obs_encoder params: {len(obs_encoder_params)}')
        param_groups = [
            {'params': self.model.model.parameters()},
            {'params': obs_encoder_params, 'lr': obs_encoder_lr}
        ]
        optimizer_cfg = OmegaConf.to_container(cfg.optimizer, resolve=True)
        optimizer_cfg.pop('_target_')
        self.optimizer = torch.optim.AdamW(
            params=param_groups,
            **optimizer_cfg
        )

        # configure training state
        self.global_step = 0
        self.epoch = 0

        # do not save optimizer if resume=False
        if not cfg.training.resume:
            self.exclude_keys = ['optimizer']

    def _get_output_path(self, cfg: OmegaConf) -> str:
        output_path = getattr(self, 'logging', None)
        if output_path is None and 'output_file' in cfg:
            output_path = cfg.output_file
        if output_path is None:
            raise ValueError(
                'Output path is not set. Provide cfg.output_file or assign workspace.logging before run().'
            )
        return output_path

    def run(self):
        cfg = copy.deepcopy(self.cfg)

        # configure dataset
        dataset: BaseImageDataset
        dataset = hydra.utils.instantiate(cfg.task.dataset)
        assert isinstance(dataset, BaseImageDataset) or isinstance(dataset, BaseDataset)
        train_dataloader = DataLoader(dataset, **cfg.dataloader)
        normalizer = dataset.get_normalizer()

        device = torch.device(cfg.training.device)
        self.ema_model.set_normalizer(normalizer)
        self.ema_model.to(device)
              
        # configure logging

        full_x = []; full_y = []
        with torch.no_grad():
            for batch_idx, batch in enumerate(train_dataloader):
                # device transfer
                batch = dict_apply(batch, lambda x: x.to(device, non_blocking=True))

                mod = self.ema_model
                nobs = mod.normalizer.normalize(batch['obs'])                    # normalized_observations
                nactions = mod.normalizer['action'].normalize(batch['action'])   # normalized_actions
                batch_size = nactions.shape[0]

                trajectory = nactions.reshape(batch_size, -1)
                global_cond = mod.obs_encoder(nobs)

                print(f'At batch {batch_idx}/{len(train_dataloader)}')
                print(f'X: {global_cond.shape}, Y: {trajectory.shape}')
                full_x.append(global_cond.cpu()); full_y.append(trajectory.cpu())
        full_x = torch.cat(full_x, dim=0)
        full_y = torch.cat(full_y, dim=0)
        print(f'Full X: {full_x.shape}, Full Y: {full_y.shape}')
        torch.save({'X': full_x, 'Y': full_y}, self.logging)



@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.parent.joinpath("config")),
    config_name=pathlib.Path(__file__).stem)
def main(cfg):
    workspace = TrainDiffusionUnetImageWorkspace(cfg)
    workspace.run()


if __name__ == "__main__":
    main()
