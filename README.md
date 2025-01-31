```math
v = \hat{v} + \text{imu\_data} \cdot dt
```
```math
init = \hat{z}[0] + v \cdot dt
```
```math
z = \hat{z}[0] + v\cdot dt
```
```math
\mathbf{z} =
\begin{bmatrix} 
z & 0 & 0 
\end{bmatrix}^T
```
```math
\hat{x}^- = F\cdot \hat{x}
```
```math
P_{k}^- = F\cdot P_{k-1}\cdot F^T + Q
```
```math
K_{k} = P_{k}^- \cdot H^T \cdot (H\cdot P_{k}^- \cdot H^T +R)^-1
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
\hat{v} = (\hat{z}[0] - init) / dt
```
