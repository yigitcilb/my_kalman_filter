This is a kalman filter to estimate 1D location using acceleration data 




```math
\hat{x}^- = F\cdot \hat{x} + \Delta t * \begin{bmatrix} speed\\ acc \end{bmatrix}

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
\hat{z} = \hat{x} + \Delta t \cdot \begin{bmatrix} speed\\acc \end{bmatrix} 
```
```math
\hat{x} = \hat{x}^- + K \cdot (z - H \cdot \hat{x}^-)
```
```math
F = \begin{bmatrix}
1 & \Delta t & 0 \\
0 & 1 & \Delta t \\
0 & 0 & 1
\end{bmatrix}
```

```math
H = \begin{bmatrix}
0 & 0 & 1
\end{bmatrix}
```
