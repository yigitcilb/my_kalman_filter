This is a kalman filter to estimate 1D location using acceleration data 




```math
\hat{x}^- = F\cdot \hat{x} + B\cdot a 
```
```math
P_{k}^- = F\cdot P_{k-1}\cdot F^T + Q
```
```math
K_{k} = P_{k}^- \cdot H^T \cdot (H\cdot P_{k}^- \cdot H^T +R)^{-1}
```
```math
P_{k} = (I - K_{k} \cdot H) \cdot P_{k}^-
```
```math
\hat{x} = \hat{x}^- + K \cdot (z - H \cdot \hat{x}^-)
```
```math
\hat{z} = H \cdot \hat{x}
```

```math
F = \begin{bmatrix}
1 & \Delta t & 0 \\
0 & 1 & \Delta t \\
0 & 0 & 1
\end{bmatrix}
```
```math
B = \begin{bmatrix}
{{\Delta t}^2}/2 & \Delta t & 1
\end{bmatrix}
```
```math
H = \begin{bmatrix}
0 & 0 & 1
\end{bmatrix}
```
