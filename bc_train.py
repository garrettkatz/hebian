import numpy as np
import torch as tr
import torch.nn as nn
import matplotlib.pyplot as plt


# ---------------- data loading ----------------

def load_split(path):
    data = np.load(path)
    joints = data["joints"].astype(np.float32)   # (N, 6) obs
    actions = data["actions"].astype(np.float32)  # (N, 6) labels (deltas)
    return tr.from_numpy(joints), tr.from_numpy(actions)


# ---------------- network ----------------

def make_net(obs_dim=6, hid_dim=64, out_dim=6):
    return nn.Sequential(
        nn.Linear(obs_dim, hid_dim),
        nn.ReLU(),
        nn.Linear(hid_dim, out_dim),
    )


# ---------------- training ----------------

def train_one_config(train_obs, train_act, test_obs, test_act,
                      hid_dim=64, lr=0.01, epochs=50, batch_size=64, log=True):
    net = make_net(obs_dim=train_obs.shape[1], hid_dim=hid_dim, out_dim=train_act.shape[1])
    optimizer = tr.optim.SGD(net.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    n = train_obs.shape[0]
    train_losses = []
    test_losses = []

    for epoch in range(epochs):
        perm = tr.randperm(n)
        epoch_loss = 0.0
        n_batches = 0

        for start in range(0, n, batch_size):
            idx = perm[start:start + batch_size]
            batch_obs = train_obs[idx]
            batch_act = train_act[idx]

            pred = net(batch_obs)
            loss = loss_fn(pred, batch_act)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()
            n_batches += 1

        train_losses.append(epoch_loss / n_batches)

        with tr.no_grad():
            test_pred = net(test_obs)
            test_loss = loss_fn(test_pred, test_act).item()
        test_losses.append(test_loss)

        if log and epoch % 5 == 0:
            print(f"  epoch {epoch:3d}  train_mse={train_losses[-1]:.6f}  test_mse={test_loss:.6f}")

    return net, train_losses, test_losses


def plot_losses(train_losses, test_losses, title="BC training", save_path="loss_curve.png"):
    plt.figure(figsize=(7, 5))
    plt.plot(train_losses, label="train MSE")
    plt.plot(test_losses, label="test MSE")
    plt.xlabel("epoch")
    plt.ylabel("MSE loss")
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    print(f"saved plot to {save_path}")


if __name__ == "__main__":
    train_obs, train_act = load_split("expert_data_train.npz")
    test_obs, test_act = load_split("expert_data_test.npz")

    print(f"train: {train_obs.shape[0]} transitions, test: {test_obs.shape[0]} transitions")

    # ---- single run first, to sanity check ----
    net, train_losses, test_losses = train_one_config(
        train_obs, train_act, test_obs, test_act,
        hid_dim=64, lr=0.01, epochs=150, batch_size=64,
    )
    plot_losses(train_losses, test_losses, title="hid_dim=64, lr=0.01", save_path="loss_curve.png")
    tr.save(net.state_dict(), "bc_net_h64_lr0.01.pt")
    print("saved trained model to bc_net_h64_lr0.01.pt")

    # ---- hyperparameter sweep ----
    results = []
    for hid_dim in [32, 64, 128]:
        for lr in [0.1, 0.01, 0.001]:
            print(f"\n=== hid_dim={hid_dim} lr={lr} ===")
            net, tr_losses, te_losses = train_one_config(
                train_obs, train_act, test_obs, test_act,
                hid_dim=hid_dim, lr=lr, epochs=150, batch_size=64, log=False,
            )
            plot_losses(tr_losses, te_losses,
                        title=f"hid_dim={hid_dim} lr={lr}",
                        save_path=f"loss_curve_h{hid_dim}_lr{lr}.png")
            results.append({
                "hid_dim": hid_dim,
                "lr": lr,
                "final_train_mse": tr_losses[-1],
                "final_test_mse": te_losses[-1],
            })

    print("\n=== sweep summary (sorted by final test MSE) ===")
    for r in sorted(results, key=lambda r: r["final_test_mse"]):
        print(f"hid_dim={r['hid_dim']:4d}  lr={r['lr']:<6}  "
              f"final_train_mse={r['final_train_mse']:.6f}  final_test_mse={r['final_test_mse']:.6f}")

    with open("sweep_results.txt", "w") as f:
        f.write("hid_dim,lr,final_train_mse,final_test_mse\n")
        for r in sorted(results, key=lambda r: r["final_test_mse"]):
            f.write(f"{r['hid_dim']},{r['lr']},{r['final_train_mse']:.6f},{r['final_test_mse']:.6f}\n")
    print("\nsaved exact numbers to sweep_results.txt")