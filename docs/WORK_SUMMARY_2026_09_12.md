# BÁO CÁO TỔNG KẾT CÔNG VIỆC
**Dự án**: Enhancing ROV Dead Reckoning Dynamics (`Enhence_DR`)  
**Nhánh làm việc**: `Dynamic_model`  
**Ngày thực hiện**: 12/09/2026  

---

## 1. MỤC TIÊU VÀ YÊU CẦU CÔNG VIỆC

Trong ngày làm việc hôm nay, các nhiệm vụ chính được đặt ra và hoàn thành bao gồm:
1. **Chuẩn hóa 3 cấu hình bậc tự do (DOF)**: Tái cấu trúc mã nguồn phù hợp với 3 cấu hình chuẩn:
   - `ROV_6DOF_FULL`: Đầy đủ 6 bậc tự do $\{u, v, w, p, q, r\}$.
   - `ROV_4DOF_CONFIG_1`: 4 bậc tự do $\{u, w, q, r\}$ (Surge, Heave, Pitch, Yaw).
   - `ROV_3DOF_CONFIG_1`: 3 bậc tự do $\{u, w, r\}$ (Surge, Heave, Yaw).
   - *Ràng buộc*: Không sửa đổi module kiểm thử `tests/`, chỉ điều chỉnh logic trong `src/` và `include/`.
2. **Rà soát chuyên sâu logic toán học**: Phân tích toàn bộ hệ thống phương trình động học (Kinematics) và động lực học (Dynamics) theo chuẩn Fossen Marine Craft để tìm ra các thiếu sót hoặc điểm chưa chuẩn xác.
3. **Triển khai và hoàn thiện logic toán học**:
   - Khử trôi dạt động học (Kinematic Drift) cho các cấu hình thu giảm.
   - Tích phân Runge-Kutta 4 (RK4) toàn phần trên không gian Lie $SE(3)$ cho cả vị trí, tư thế quaternion và vận tốc.
   - Hoàn thiện ma trận cản phi tuyến bậc hai $6 \times 6$ có ghép kênh chéo thủy động lực học.
   - Bổ sung lực quán tính khối lượng gia tăng do dòng chảy quay trong hệ thân tàu ($M_A \dot{\nu}_c^b$).
   - Xây dựng ma trận Jacobian thu giảm $J_r(\eta)$.
   - Chiếu khử triệt để vận tốc dư bằng ma trận chiếu $P_{active} = T^T T$.
   - Mô hình trễ động học bậc 1 cho động cơ chân vịt (Thruster Motor Lag).

---

## 2. CHI TIẾT CÁC HẠNG MỤC CÔNG VIỆC ĐÃ THỰC HIỆN

```
+---------------------------------------------------------------------------------------------------------+
|                                    CÁC HẠNG MỤC HOÀN THÀNH TRONG NGÀY                                   |
+------------------------------------+------------------------------------+-------------------------------+
|       1. CẤU HÌNH & CHUYỂN ĐỔI DOF |            2. ĐỘNG HỌC             |         3. ĐỘNG LỰC HỌC       |
+------------------------------------+------------------------------------+-------------------------------+
| • Enum 3 cấu hình ROV chuẩn        | • Khử trôi roll ở 4-DOF: phi = 0   | • Cản bậc hai D_q 6x6 ghép chéo|
| • Factory methods DofTransformer   | • Chuẩn hóa quaternion phẳng 3-DOF | • Lực dòng chảy quay M_A*nu_c |
| • Bộ đọc YAML tolerant parser      | • Jacobian thu giảm J_r(eta)       | • Chiếu vận tốc: nu <- T^T*T*nu|
| • RosAdapter POD 4-DOF & 6-DOF     | • RK4 toàn phần SE(3) O(dt^4)      | • Trễ động cơ đẩy tau_m       |
+------------------------------------+------------------------------------+-------------------------------+
```

### 2.1. Chuẩn Hóa và Nâng Cấp Hệ Thống Chuyển Đổi DOF

- **Chuẩn hóa `ROV_4DOF_CONFIG_1`**:
  - Ban đầu cấu hình 4-DOF trong mã nguồn sử dụng $\{u, v, w, r\}$ (Surge, Sway, Heave, Yaw).
  - Đã tái cấu trúc chuẩn hóa lại thành $\{u, w, q, r\}$ (Surge, Heave, Pitch, Yaw), phù hợp với mô hình ROV lặn sâu có điều khiển góc chúi ngóc và quay hướng.
- **Mở rộng `DofTransformer` và `DofConfig`**:
  - Bổ sung enum: `DofPreset::ROV_6DOF_FULL`, `DofPreset::ROV_4DOF_CONFIG_1`, `DofPreset::ROV_3DOF_CONFIG_1`.
  - Bổ sung các static factory methods: `make_6dof()`, `make_rov_4dof()`, `make_rov_3dof()`, `make_planar_3dof()`.
- **Tối ưu hóa bộ nạp cấu hình `ConfigLoader`**:
  - Hỗ trợ nạp ma trận tùy chỉnh từ YAML (`transform_matrix_4dof_1`, `transform_matrix_3dof_1`).
  - Xử lý cơ chế đọc linh hoạt (tolerant parsing): duyệt toàn bộ map để lấy cấu hình preset khai báo cuối cùng (last-write-wins), tránh xung đột khi file cấu hình có nhiều khối preset dự phòng.
- **Nâng cấp `RosAdapter`**:
  - Thêm các hàm chuyển đổi `twist_pod_from_4dof(u, w, q, r)` và `twist_pod_from_6dof(...)` để giao tiếp với hệ sinh thái ROS.

---

### 2.2. Hoàn Thiện Logic Toán Học Động Học (Kinematics)

#### A. Khử trôi dạt góc roll do hiệu ứng hình học $SO(3)$ (Kinematic Drift Elimination)
- **Vấn đề toán học**:
  - Khi ROV hoạt động ở 4-DOF $\{u, w, q, r\}$, vận tốc roll body $p = 0$.
  - Tuy nhiên, theo phương trình vi phân góc Euler:
    $$\dot{\phi} = p + (q \sin\phi + r \cos\phi)\tan\theta$$
  - Khi $\phi = 0$ và $p = 0$: $\dot{\phi} = r \tan\theta \neq 0$. Do tính chất phi toàn ký của nhóm Lie $SO(3)$, khi ROV vừa ngóc/chúi ($\theta \neq 0$) vừa bẻ lái yaw ($r \neq 0$), góc roll $\phi$ sẽ tự động bị lệch khỏi 0 nếu chỉ tích phân quaternion thông thường.
- **Giải pháp đã thực hiện**:
  - Triển khai hàm `enforce_4dof_constraints()` trong `KinematicState`: trích xuất $(\theta, \psi)$, cưỡng bức đặt $\phi = 0$, sau đó tái cấu trúc quaternion:
    $$q = \left[\cos\frac{\theta}{2}\cos\frac{\psi}{2}, -\sin\frac{\theta}{2}\sin\frac{\psi}{2}, \sin\frac{\theta}{2}\cos\frac{\psi}{2}, \cos\frac{\theta}{2}\sin\frac{\psi}{2}\right]^T$$
  - Triển khai hàm `enforce_3dof_constraints()` cho 3-DOF: cưỡng bức $\phi = 0, \theta = 0$, chuẩn hóa quaternion phẳng $q = [\cos(\psi/2), 0, 0, \sin(\psi/2)]^T$.

#### B. Ma trận Jacobian động học thu giảm $J_r(\eta)$
- Cung cấp hàm tổng quát `transform_jacobian(J)`: $J_r = T J T^T$.
- Triển khai hàm `compute_reduced_jacobian(state)` trả về ma trận Jacobian tương ứng với từng cấu hình DOF.
- Bổ sung hàm giải tích tĩnh `compute_3dof_jacobian(psi)` cho 3-DOF $\{u, w, r\} \rightarrow \{\dot{x}, \dot{z}, \dot{\psi}\}$:
  $$J_{3dof}(\psi) = \begin{bmatrix} \cos\psi & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

#### C. Tích phân Runge-Kutta 4 (RK4) toàn phần trên $SE(3)$
- Trước đây: `step_rk4` tính RK4 cho vận tốc $\nu$, nhưng vị trí $p_{ned}$ và quaternion $q$ lại cập nhật bằng Euler bậc 1 ở cuối bước.
- Cải tiến: Nâng cấp thành thuật toán tích phân RK4 toàn phần đồng bộ cấp chính xác $O(\Delta t^4)$ cho cả 12 biến trạng thái:
  - Giai đoạn $k_i$: Đạo hàm vị trí $\dot{p}_i = R_b^n(q_i) v_i$, đạo hàm quaternion $\dot{q}_i = \frac{1}{2} q_i \otimes \begin{bmatrix} 0 \\ \omega_i \end{bmatrix}$, đạo hàm vận tốc $\dot{\nu}_i = f(\text{state}_i, \tau)$.
  - Cập nhật trung bình có trọng số RK4: $\frac{1}{6}(k_1 + 2k_2 + 2k_3 + k_4)$.
  - Tự động chuẩn hóa quaternion $\|q\| = 1$ và áp dụng các ràng buộc động học ở cuối bước.

---

### 2.3. Hoàn Thiện Logic Toán Học Động Lực Học (Dynamics)

#### A. Ma trận cản bậc hai tổng quát $6 \times 6$ (Quadratic Damping with Cross-Coupling)
- Trong `DampingMatrixEvaluator::compute_D_quadratic`:
  - Trước đây: chỉ nhân các phần tử trên đường chéo chính: $D_q(i, i) = (D_q)_{ii} |\nu_{r, i}|$.
  - Đã nâng cấp: tính toán trên toàn bộ ma trận $6 \times 6$:
    $$D_q(i, j) = (D_q)_{ij} \cdot |\nu_{r, j}|$$
  - Giúp mô phỏng trọn vẹn các hiệu ứng ghép kênh thủy động lực học thực tế như cản sway sinh mô-men yaw ($N_{|v|v}$) hoặc cản surge sinh mô-men pitch ($M_{|u|u}$).

#### B. Lực quán tính chất lỏng gia tăng do dòng chảy quay ($M_A \dot{\nu}_c^b$)
- Theo phương trình chuyển động Fossen đầy đủ:
  $$(M_{RB} + M_A)\dot{\nu} = \tau - C_{RB}(\nu)\nu - C_A(\nu_r)\nu_r - D(\nu_r)\nu_r - g(\eta) + M_A \dot{\nu}_c^b$$
- Khi thân tàu quay với vận tốc góc $\omega = [p, q, r]^T$, vận tốc dòng chảy trong hệ thân tàu $\nu_c^b = R_b^{n\,T} v_c^{ned}$ sẽ biến thiên theo thời gian:
  $$\dot{\nu}_c^b = \begin{bmatrix} -\omega \times \nu_c^b \\ 0_{3\times 1} \end{bmatrix} + R_b^{n\,T} \dot{v}_c^{ned}$$
- Đã bổ sung hàm `compute_current_acceleration` trong `FluidCurrent` và tích hợp lực $+ M_A \dot{\nu}_c^b$ vào cả bài toán động lực học thuận (`compute_forward_dynamics`) và nghịch (`compute_inverse_dynamics`).

#### C. Khử vận tốc dư trên không gian thu giảm
- Thêm ma trận chiếu $P_{active} = T^T T$ ($6 \times 6$) vào `DofTransformer`.
- Tự động chiếu vận tốc $\nu \leftarrow P_{active} \nu$ trong `DynamicModel::apply_kinematic_constraints` để loại bỏ hoàn toàn các thành phần vận tốc ban đầu ngoài trục hoạt động.

#### D. Mô hình trễ động học động cơ chân vịt (Thruster Lag)
- Thêm trường `time_constant` ($\tau_{m, i}$, đơn vị giây, mặc định 0.0s) vào cấu trúc `ThrusterUnit`.
- Triển khai hàm `step_thruster_dynamics(target_thrusts, dt)` mô phỏng độ trễ đáp ứng cơ học và điện học của động cơ BLDC:
  $$\dot{T}_i = \frac{1}{\tau_{m, i}} (T_{target, i} - T_i)$$

---

## 3. DANH MỤC CÁC TỆP ĐÃ CHỈNH SỬA

| STT | Tệp tin | Vị trí | Tóm tắt các thay đổi |
|---|---|---|---|
| 1 | `include/nav_dynamics/dof_transformer.hpp` | Header | Bổ sung enum 3 presets, factory methods, ma trận chiếu $P_{active}$, các hàm tính Jacobian thu giảm $J_r$. |
| 2 | `src/dof_transformer.cpp` | Source | Cài đặt `ROV_4DOF_CONFIG_1` chuẩn $\{u, w, q, r\}$, cài đặt các hàm factory và tính toán Jacobian 3-DOF / thu giảm. |
| 3 | `include/nav_dynamics/dof_config.hpp` | Header | Khai báo các factory methods đồng bộ với `DofTransformer`. |
| 4 | `src/dof_config.cpp` | Source | Triển khai factory methods tương thích ngược và 3 presets mới. |
| 5 | `include/nav_dynamics/types.hpp` | Header | Bổ sung `compute_current_acceleration` cho dòng chảy, bổ sung `enforce_3dof_constraints` và `enforce_4dof_constraints`. |
| 6 | `src/damping_matrix.cpp` | Source | Nâng cấp ma trận cản phi tuyến bậc hai $D_q$ tính đầy đủ $6 \times 6$ cross-coupling. |
| 7 | `include/nav_dynamics/dynamic_model.hpp` | Header | Khai báo phương thức `apply_kinematic_constraints` và hỗ trợ lực quán tính dòng chảy. |
| 8 | `src/dynamic_model.cpp` | Source | Thêm lực $+ M_A \dot{\nu}_c^b$, áp đặt ràng buộc chiếu vận tốc, nâng cấp tích phân toàn phần RK4 trên $SE(3)$. |
| 9 | `src/config_loader.cpp` | Source | Hỗ trợ nạp preset linh hoạt từ YAML, ưu tiên đúng ma trận tùy chỉnh, giải quyết duplicate keys. |
| 10 | `include/nav_dynamics/thruster_allocation.hpp` | Header | Bổ sung trường `time_constant` và khai báo `step_thruster_dynamics`. |
| 11 | `src/thruster_allocation.cpp` | Source | Cài đặt mô phỏng trễ động cơ đẩy bậc 1 theo thời gian rời rạc. |
| 12 | `include/nav_dynamics/ros_adapter.hpp` | Header | Bổ sung hàm tạo `TwistPOD` cho 4-DOF và 6-DOF. |
| 13 | `src/ros_adapter.cpp` | Source | Cài đặt chuyển đổi POD cho 4-DOF và 6-DOF. |

---

## 4. KẾT QUẢ KIỂM THỬ VÀ XÁC MINH

### 4.1. Biên dịch hệ thống (Build)
- **Công cụ**: CMake 3.28 + GCC 11.4 (C++17)
- **Lệnh thực hiện**: `cmake --build build`
- **Kết quả**: Thành công tuyệt đối, **0 errors, 0 warnings**.

### 4.2. Kiểm thử tự động (Unit Tests - CTest)
- **Lệnh thực hiện**: `ctest --test-dir build --output-on-failure`
- **Kết quả**: **100% Tests Passed (9/9 tests)**
  ```
  Test project /home/cnciot/WorkSpace/Dev/ROV/Enhence_DR/build
      Start 1: test_mass_matrix
  1/9 Test #1: test_mass_matrix ......................   Passed    0.00 sec
      Start 2: test_coriolis_matrix
  2/9 Test #2: test_coriolis_matrix ..................   Passed    0.00 sec
      Start 3: test_damping_matrix
  3/9 Test #3: test_damping_matrix ...................   Passed    0.00 sec
      Start 4: test_restoring_force
  4/9 Test #4: test_restoring_force ..................   Passed    0.00 sec
      Start 5: test_dof_reduction
  5/9 Test #5: test_dof_reduction ....................   Passed    0.00 sec
      Start 6: test_dynamic_model
  6/9 Test #6: test_dynamic_model ....................   Passed    0.01 sec
      Start 7: test_dof_transformer
  7/9 Test #7: test_dof_transformer ..................   Passed    0.00 sec
      Start 8: test_config_loader
  8/9 Test #8: test_config_loader ....................   Passed    0.04 sec
      Start 9: test_dof_transformation_integration ...   Passed    0.01 sec

  100% tests passed, 0 tests failed out of 9
  ```

### 4.3. Mô phỏng thực tế (Simulation Example)
- **Lệnh thực hiện**: `./build/examples/rov_simulation_example`
- **Kết quả**: Chương trình mô phỏng hoàn thành trọn vẹn, các giá trị lực điều khiển, phân bổ chân vịt và quỹ đạo chuyển động hội tụ ổn định, không có hiện tượng gián đoạn hay trôi dạt góc.

---

## 5. KẾT LUẬN

Hệ thống đã đạt độ hoàn thiện cao cả về:
1. Tính nhất quán trong kiến trúc mã nguồn (hỗ trợ chuyển đổi mượt mà giữa 6-DOF, 4-DOF và 3-DOF).
2. Độ chính xác toán học chặt chẽ theo lý thuyết thủy động lực học tàu lặn Fossen Marine Craft.
3. Độ ổn định số học trong tích phân lâu dài nhờ thuật toán RK4 toàn phần trên $SE(3)$ và cơ chế áp đặt ràng buộc hình học.
