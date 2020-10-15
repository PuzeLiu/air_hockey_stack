import torch

A = torch.tensor([[0., 1., 2., 3.]])


def my_nullspace(At, rcond=None):

    ut, st, vht = torch.Tensor.svd(At, some=False, compute_uv=True)
    vht=vht.T
    Mt, Nt = ut.shape[0], vht.shape[1]
    if rcond is None:
        rcondt = torch.finfo(st.dtype).eps * max(Mt, Nt)
    tolt = torch.max(st) * rcondt
    numt= torch.sum(st > tolt, dtype=int)
    nullspace = vht[numt:,:].T.cpu().conj()
    # nullspace.backward(torch.ones_like(nullspace),retain_graph=True)
    return nullspace

A.requires_grad_()
out = my_nullspace(A)

print(out)
print(A @ (torch.rand(1) * out[:, 0]))
print(A @ (torch.rand(1) * out[:, 1]))
print(A @ (torch.rand(1) * out[:, 2]))

out.sum().backward()
print("A.grad", A.grad)
