import torch
import time


def fun(x):
    return x**2

t_start = time.time()
for i in range(10000):
    x = torch.ones(100).requires_grad_()
    y = fun(x)
    jac = torch.zeros(y.shape[0], x.shape[0])
    for i in range(y.shape[0]):
        jac[i] = torch.autograd.grad(y[i], x, retain_graph=True)[0]
print(jac)
print("Forward Backward Time:", time.time()-t_start)

t_start = time.time()
for i in range(10000):
    x = torch.ones(100)
    y = fun(x)
    jac = torch.autograd.functional.jacobian(fun, x)
print(jac)
print("Functional Jacobian Time:", time.time() - t_start)
