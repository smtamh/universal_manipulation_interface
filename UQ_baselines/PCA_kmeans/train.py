import torch
import sys
master_dir = '../../UQ_baselines'
sys.path.append(master_dir)
import data_loader
from argparse import ArgumentParser
from net_PCA import PCAKMeansNet
device = torch.device(f"cuda" if torch.cuda.is_available() else "cpu")

    
parser = ArgumentParser()
parser.add_argument("--data-file", default=None, type=str)
parser.add_argument("--emb-dim", default=64, type=int)
args = parser.parse_args()

if __name__ == "__main__":
    X, Y = data_loader.get_data(data_file=args.data_file, adjust_shape=False)
    emb_dim = min(args.emb_dim, X.shape[1])
    net = PCAKMeansNet(X, emb_dim=emb_dim).to(device)
    # Save the model state
    ckpt_file = 'umi_PCA_kmeans.ckpt'
    ckpt = {
        'model': net.state_dict()
    }
    torch.save(ckpt, ckpt_file)
    
