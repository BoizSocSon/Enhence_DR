# nav_dynamics — C++ 6-DOF Underwater & Marine Dynamic Model Library

Thư viện C++ độc lập bên thứ ba (Third-party C++17 Library) phục vụ tính toán **Mô hình Động lực học** (Dynamic Model) cho ROV, AUV và phương tiện hàng hải:
- **Zero ROS Dependency**: Không chứa bất kỳ dependency hay header nào từ ROS/ROS 2 trong nhân thư viện.
- **Chuẩn hóa I/O cho ROS**: Cung cấp module `ros_adapter` với các cấu trúc POD (`TwistPOD`, `WrenchPOD`, `PosePOD`, `OdometryPOD`) và duck-typed templates cho phép copy/serialize trực tiếp sang ROS messages chỉ với 1 dòng code.
- **Module hóa từng thành phần ma trận**: Mỗi ma trận trong phương trình vi phân động lực học Fossen được viết và xử lý trong **cặp mã nguồn riêng biệt** (`.hpp` và `.cpp`).
- **Module chuyên biệt Ma trận Chuyển đổi DOF (`DofTransformer`)**: Sinh ma trận chuyển đổi $\mathbf{T} \in \mathbb{R}^{n \times 6}$ thỏa mãn $\mathbf{T}\mathbf{T}^T = \mathbf{I}$, thực hiện các phép nhân ma trận Fossen trực tiếp để giảm số chiều: $\mathbf{T}\mathbf{M}\mathbf{T}^T$, $\mathbf{T}\mathbf{C}\mathbf{T}^T$, $\mathbf{T}\mathbf{D}\mathbf{T}^T$, $\mathbf{T}\mathbf{g}$, $\mathbf{T}\boldsymbol{\tau}$.
- **File Cấu hình YAML Tích hợp ArduSub (`config/rov_params.yaml` & `ConfigLoader`)**: Quản lý toàn bộ tham số vật lý ROV, hệ số motor matrix của ArduSub, IMU lever arms và môi trường NED.

---

## 1. Phương trình Động lực học Toán học

Phương trình Fossen 6-DOF trong Body-fixed frame:
$$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu}, \boldsymbol{\nu}_r) \boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu}_r) \boldsymbol{\nu}_r + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}$$

Khi giảm chiều sang không gian $n$-DOF ($n \le 6$) qua ma trận chuyển đổi $\mathbf{T} \in \mathbb{R}^{n \times 6}$:
$$\mathbf{M}_r \dot{\boldsymbol{\nu}}_r + \mathbf{C}_r \boldsymbol{\nu}_r + \mathbf{D}_r \boldsymbol{\nu}_r + \mathbf{g}_r = \boldsymbol{\tau}_r$$
với:
- $\mathbf{M}_r = \mathbf{T} \mathbf{M} \mathbf{T}^T \in \mathbb{R}^{n \times n}$
- $\mathbf{C}_r = \mathbf{T} \mathbf{C} \mathbf{T}^T \in \mathbb{R}^{n \times n}$
- $\mathbf{D}_r = \mathbf{T} \mathbf{D} \mathbf{T}^T \in \mathbb{R}^{n \times n}$
- $\mathbf{g}_r = \mathbf{T} \mathbf{g} \in \mathbb{R}^{n \times 1}$
- $\boldsymbol{\tau}_r = \mathbf{T} \boldsymbol{\tau} \in \mathbb{R}^{n \times 1}$

---

## 2. Cấu trúc Mã nguồn Thư viện

```
Navigation_System_Library/
├── CMakeLists.txt                      # Cấu hình CMake độc lập (yêu cầu C++17, Eigen3, yaml-cpp)
├── README.md                           # Tài liệu hướng dẫn sử dụng và tích hợp
├── config/
│   └── rov_params.yaml                 # File cấu hình tham số cố định ROV, ArduSub và NED
├── include/
│   └── nav_dynamics/
│       ├── types.hpp                   # Vector6d, Matrix6d, State, Wrench, VehicleParameters
│       ├── dof_transformer.hpp         # Module chuyên biệt ma trận chuyển đổi DOF (T * M * T^T)
│       ├── dof_config.hpp              # Quản lý cấu hình DOF và tương thích ngược
│       ├── config_loader.hpp           # Bộ nạp cấu hình YAML tích hợp ArduSub
│       ├── mass_matrix.hpp             # Module Ma trận M (M_RB + M_A)
│       ├── coriolis_matrix.hpp         # Module Ma trận Coriolis C(nu)
│       ├── damping_matrix.hpp          # Module Ma trận Cản D(nu_r)
│       ├── restoring_force.hpp         # Module Vector Hồi phục g(eta)
│       ├── thruster_allocation.hpp     # Module Phân bổ Lực đẩy B & PWM mapping
│       ├── dynamic_model.hpp           # Module Tích hợp Forward/Inverse Dynamics & Solver RK4
│       └── ros_adapter.hpp             # Module Chuẩn hóa I/O cho ROS (POD & Templates)
├── src/
│   ├── dof_transformer.cpp
│   ├── dof_config.cpp
│   ├── config_loader.cpp
│   ├── mass_matrix.cpp
│   ├── coriolis_matrix.cpp
│   ├── damping_matrix.cpp
│   ├── restoring_force.cpp
│   ├── thruster_allocation.cpp
│   ├── dynamic_model.cpp
│   └── ros_adapter.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── test_common.hpp                 # Macro test assertions độc lập với NDEBUG
│   ├── test_mass_matrix.cpp            # Test đối xứng, xác định dương, added mass
│   ├── test_coriolis_matrix.cpp        # Test skew-symmetry, triệt tiêu 3-DOF
│   ├── test_damping_matrix.cpp         # Test tính tiêu tán, bậc 2 phi tuyến
│   ├── test_restoring_force.cpp        # Test cân bằng trọng lực - lực nổi
│   ├── test_dof_reduction.cpp          # Test ma trận chiếu P và bảo toàn động học
│   ├── test_dynamic_model.cpp          # Test solver RK4, terminal velocity và POD conversions
│   ├── test_dof_transformer.cpp        # Test chuyên sâu module DofTransformer (T*M*T^T)
│   └── test_config_loader.cpp          # Test nạp file rov_params.yaml và ArduSub config
└── examples/
    ├── CMakeLists.txt
    └── rov_simulation_example.cpp      # Ví dụ mô phỏng ROV nạp từ config/rov_params.yaml
```

---

## 3. Bản chất Tham số Body Frame vs NED Frame

| Nhóm tham số | Hệ quy chiếu | Các đại lượng chính | Cách xác định / Nguồn lấy |
|---|---|---|---|
| **Tham số thân vỏ ROV** | **Body frame $\{b\}$** | Khối lượng $m$, Tensor quán tính $\mathbf{I}_b$, Tâm khối lượng $\mathbf{r}_G$, Tâm nổi $\mathbf{r}_B$, Added mass $\mathbf{M}_A$, Damping $\mathbf{D}_l, \mathbf{D}_q$, Vị trí thrusters $\mathbf{r}_j$, Motor factors ArduSub | Cố định với ROV. Lấy từ file CAD, thí nghiệm cân kéo hoặc thông số firmware ArduSub. |
| **Tham số môi trường & toàn cục** | **NED frame $\{n\}$** | Gia tốc trọng trường $\mathbf{g}^n = [0, 0, 9.80665]^T$, Lực nổi $\mathbf{f}_b^n = [0, 0, -\rho g \nabla]^T$, Dòng chảy ngầm $\mathbf{v}_c^n$, Từ trường $\mathbf{m}^n$ | Thuộc về môi trường tự nhiên. Đo độ mặn nước để tính $\rho$, dòng chảy $\mathbf{v}_c^n$ ước lượng online qua bộ lọc InEKF/ESKF, từ trường lấy từ World Magnetic Model (WMM). |

---

## 4. Hướng dẫn Biên dịch & Chạy Test

### Lệnh Biên dịch:
```bash
cd /home/stevehoang/Navigation_System_Library
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Chạy Toàn bộ 8 Unit Tests:
```bash
ctest --output-on-failure
```
Kết quả:
```text
100% tests passed, 0 tests failed out of 8
```

### Chạy Ví dụ Mô phỏng Nạp Cấu hình YAML:
```bash
./examples/rov_simulation_example
```

---

## 5. Quy Chuẩn Phát Triển & Quản Lý Mã Nguồn (Git Conventions)

Dự án áp dụng chặt chẽ quy chuẩn **Conventional Commits** và chiến lược phân nhánh **Feature Branching**:
- 📖 **Tài liệu chi tiết**: [docs/GIT_WORKFLOW.md](docs/GIT_WORKFLOW.md)
- 📝 **Commit Message Template**: [`.gitmessage`](.gitmessage)
- 🔀 **Pull Request Template**: [`.github/PULL_REQUEST_TEMPLATE.md`](.github/PULL_REQUEST_TEMPLATE.md)
- 🪝 **Tự động kiểm tra cú pháp commit**: Cài đặt hook bằng lệnh `git config core.hooksPath .githooks`
