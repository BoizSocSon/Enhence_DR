# HƯỚNG DẪN SỬ DỤNG MÃ NGUỒN THƯ VIỆN ĐỘNG LỰC HỌC VÀ ĐỘNG HỌC (NAV_DYNAMICS)
**Đặc tả tích hợp n-DOF, Hướng dẫn Build & Tích hợp Thư viện, Trích xuất Dữ liệu Đầu ra**

---

## MỤC LỤC
1. [Tổng quan Kiến trúc Thư viện](#1-tổng-quan-kiến-trúc-thư-viện)
2. [Cấu hình và Mở rộng n Bậc Tự do (n-DOF)](#2-cấu-hình-và-mở-rộng-n-bậc-tự-do-n-dof)
   - [2.1. Bản chất toán học của cơ chế thu giảm n-DOF](#21-bản-chất-toán-học-của-cơ-chế-thu-giảm-n-dof)
   - [2.2. Cách 1: Cấu hình qua tệp YAML (Không cần sửa mã C++)](#22-cách-1-cấu-hình-qua-tệp-yaml-khuyến-nghị---không-cần-sửa-mã-c)
   - [2.3. Cách 2: Khởi tạo động trong Runtime bằng mã C++](#23-cách-2-khởi-tạo-động-trong-runtime-bằng-mã-c)
   - [2.4. Cách 3: Thêm cấu hình Preset n-DOF cố định vào mã nguồn thư viện](#24-cách-3-thêm-cấu-hình-preset-n-dof-cố-định-vào-mã-nguồn-thư-viện)
   - [2.5. Bảng tổng hợp các hàm logic cốt lõi cho n-DOF](#25-bảng-tổng-hợp-các-hàm-logic-cốt-lõi-cho-n-dof)
3. [Hướng dẫn Build và Tích hợp Thư viện](#3-hướng-dẫn-build-và-tích-hợp-thư-viện)
   - [3.1. Yêu cầu môi trường & Thư viện phụ thuộc](#31-yêu-cầu-môi-trường--thư-viện-phụ-thuộc)
   - [3.2. Build thư viện độc lập (Standalone Build)](#32-build-thư-viện-độc-lập-standalone-build)
   - [3.3. Tích hợp vào dự án khác bằng CMake (`add_subdirectory`)](#33-tích-hợp-vào-dự-án-khác-bằng-cmake-add_subdirectory)
   - [3.4. Tích hợp tự động qua CMake `FetchContent`](#34-tích-hợp-tự-động-qua-cmake-fetchcontent)
   - [3.5. Lưu ý quan trọng khi biên dịch](#35-lưu-ý-quan-trọng-khi-biên-dịch)
4. [Trích xuất Dữ liệu Đầu ra từ Phương trình Động lực học & Động học](#4-trích-xuất-dữ-liệu-đầu-ra-từ-phương-trình-động-lực-học--động-học)
   - [4.1. Lấy gia tốc từ Động lực học thuận (Forward Dynamics)](#41-lấy-gia-tốc-từ-động-lực-học-thuận-forward-dynamics)
   - [4.2. Lấy phân tích chi tiết toàn bộ các thành phần lực (`DynamicBreakdown`)](#42-lấy-phân-tích-chi-tiết-toàn-bộ-các-thành-phần-lực-dynamicbreakdown)
   - [4.3. Truy xuất trực tiếp các ma trận hệ thống (M, C, D, g, B)](#43-truy-xuất-trực-tiếp-các-ma-trận-hệ-thống-m-c-d-g-b)
   - [4.4. Tính lực điều khiển yêu cầu từ Động lực học nghịch (Inverse Dynamics)](#44-tính-lực-điều-khiển-yêu-cầu-từ-động-lực-học-nghịch-inverse-dynamics)
   - [4.5. Lấy dữ liệu tư thế và vị trí từ Phương trình Động học (Kinematics)](#45-lấy-dữ-liệu-tư-thế-và-vị-trí-từ-phương-trình-động-học-kinematics)
   - [4.6. Trích xuất lực đẩy động cơ và tín hiệu PWM](#46-trích-xuất-lực-đẩy-động-cơ-và-tín-hiệu-pwm)
   - [4.7. Chuẩn hóa dữ liệu sang kiểu ROS POD (Zero ROS Dependency)](#47-chuẩn-hóa-dữ-liệu-sang-kiểu-ros-pod-zero-ros-dependency)
5. [Ví dụ Hoàn chỉnh Tích hợp End-to-End](#5-ví-dụ-hoàn-chỉnh-tích-hợp-end-to-end)

---

## 1. TỔNG QUAN KIẾN TRÚC THƯ VIỆN

Thư viện `nav_dynamics` mô hình hóa chuyển động của phương tiện dưới nước (ROV/AUV) theo chuẩn quốc tế **Thor I. Fossen (2011)**. Hệ thống bao gồm hai phương trình nền tảng:

1. **Phương trình Động lực học (Kinetics)** trong hệ quy chiếu Thân tàu (Body-fixed frame $\{b\}$):
   $$M \dot{\nu} + C(\nu)\nu + D(\nu_r)\nu_r + g(\eta) = \tau + \tau_{\text{current\_acc}}$$
   - $\nu = [u, v, w, p, q, r]^T$: Vận tốc 6-DOF trong hệ thân tàu.
   - $\nu_r = \nu - \nu_c$: Vận tốc tương đối đối với dòng chảy đại dương.
   - $M = M_{RB} + M_A$: Ma trận quán tính tổng (vật rắn + khối lượng gia tăng thủy động).
   - $C(\nu) = C_{RB}(\nu) + C_A(\nu_r)$: Ma trận Coriolis và hướng tâm.
   - $D(\nu_r) = D_l + D_q(|\nu_r|)$: Ma trận cản thủy động học (tuyến tính + phi tuyến bậc 2).
   - $g(\eta)$: Véc-tơ lực và mô-men hồi phục thủy tĩnh (Trọng lực $W$ và Lực nổi $B$).
   - $\tau = B u_{\text{thrust}}$: Véc-tơ lực và mô-men do các động cơ đẩy tạo ra.

2. **Phương trình Động học (Kinematics)** ánh xạ giữa hệ $\{b\}$ và hệ Địa phương NED $\{n\}$ (North-East-Down):
   $$\dot{\eta} = J(\eta)\nu = \begin{bmatrix} R_b^n(\Theta) & 0_{3\times 3} \\ 0_{3\times 3} & T_{\Theta}(\Theta) \end{bmatrix} \nu$$
   - $\eta = [p_{ned}^T, \Theta^T]^T = [x, y, z, \phi, \theta, \psi]^T$.
   - $R_b^n$: Ma trận quay biến đổi vận tốc tịnh tiến sang hệ NED.
   - $T_{\Theta}$: Ma trận biến đổi vận tốc góc sang đạo hàm góc Euler.

Thư viện được module hóa thành các lớp độc lập:
- `DofTransformer`: Quản lý phép chiếu và thu giảm không gian trạng thái $n$-DOF.
- `MassMatrixEvaluator`: Tính ma trận khối lượng $M = M_{RB} + M_A$.
- `CoriolisMatrixEvaluator`: Tính ma trận $C(\nu)$.
- `DampingMatrixEvaluator`: Tính ma trận cản $D(\nu_r)$.
- `RestoringForceEvaluator`: Tính véc-tơ phục hồi $g(\eta)$.
- `ThrusterAllocation`: Quản lý ma trận phân bổ $B$, giải nghịch đảo lực đẩy và xuất PWM.
- `DynamicModel`: Bộ điều phối tích hợp toàn diện (Forward Dynamics, Inverse Dynamics, RK4/Euler integration).
- `ConfigLoader`: Nạp cấu hình từ YAML.
- `RosAdapter`: Đóng gói dữ liệu ra các kiểu POD tương thích `geometry_msgs` và `nav_msgs` mà không cần cài ROS.

---

## 2. CẤU HÌNH VÀ MỞ RỘNG n BẬC TỰ DO (n-DOF)

### 2.1. Bản chất toán học của cơ chế thu giảm n-DOF

Mô hình 6-DOF đầy đủ có vector trạng thái $\nu \in \mathbb{R}^6$ tương ứng với 6 chỉ số `DofIndex`:
```cpp
enum class DofIndex : uint8_t {
    SURGE = 0, // Tiến/lùi theo trục x
    SWAY  = 1, // Dạt ngang theo trục y
    HEAVE = 2, // Chìm/nổi theo trục z
    ROLL  = 3, // Lắc ngang quanh trục x
    PITCH = 4, // Chúi/ngóc quanh trục y
    YAW   = 5  // Quay trở quanh trục z
};
```

Một cấu hình $n$-DOF bất kỳ ($1 \le n \le 6$) được định nghĩa bởi một ma trận biến đổi trực giao $T \in \mathbb{R}^{n \times 6}$ thỏa mãn điều kiện trực giao phải $T \cdot T^T = I_n$. 
Các phương trình thu giảm trong thư viện được thực hiện như sau:
- **Thu giảm ma trận hệ thống**:
  $$M_r = T \cdot M \cdot T^T \in \mathbb{R}^{n \times n}$$
  $$C_r = T \cdot C \cdot T^T \in \mathbb{R}^{n \times n}$$
  $$D_r = T \cdot D \cdot T^T \in \mathbb{R}^{n \times n}$$
- **Thu giảm véc-tơ lực và vận tốc**:
  $$\nu_r = T \cdot \nu \in \mathbb{R}^{n \times 1}$$
  $$\tau_r = T \cdot \tau \in \mathbb{R}^{n \times 1}, \quad g_r = T \cdot g \in \mathbb{R}^{n \times 1}$$
- **Thu giảm ma trận phân bổ lực đẩy** ($k$ động cơ):
  $$B_r = T \cdot B \in \mathbb{R}^{n \times k}$$
- **Mở rộng véc-tơ thu giảm về 6D**:
  $$\nu_{6D} = T^T \nu_r + (I_6 - T^T T)\nu_{\text{constrained}}$$

---

### 2.2. Cách 1: Cấu hình qua tệp YAML (Khuyến nghị - KHÔNG CẦN SỬA MÃ C++)

Nhờ thiết kế động lực học tổng quát của lớp `DofTransformer` và bộ nạp `ConfigLoader`, người dùng **không cần viết lại hay biên dịch lại bất kỳ hàm C++ nào** khi muốn thay đổi sang một cấu hình $n$-DOF mới. Bạn chỉ cần chỉnh sửa tệp cấu hình YAML.

#### Ví dụ 1: Cấu hình 3-DOF {Surge, Heave, Yaw} (ROV tiêu chuẩn)
Trong `config/rov_params.yaml`:
```yaml
dof_config:
  preset: "ROV_3DOF_CONFIG_1"
  active_dofs:
    - "SURGE"
    - "HEAVE"
    - "YAW"
  transform_matrix:
    - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Hàng 0: Surge (u)
    - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0] # Hàng 1: Heave (w)
    - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # Hàng 2: Yaw (r)
```

#### Ví dụ 2: Cấu hình 4-DOF {Surge, Sway, Heave, Yaw} (AUV/ROV có cơ cấu dạt ngang)
Chỉ cần sửa file YAML thành:
```yaml
dof_config:
  preset: "CUSTOM_4DOF"
  active_dofs:
    - "SURGE"
    - "SWAY"
    - "HEAVE"
    - "YAW"
  transform_matrix:
    - [1.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Surge
    - [0.0, 1.0, 0.0, 0.0, 0.0, 0.0] # Sway
    - [0.0, 0.0, 1.0, 0.0, 0.0, 0.0] # Heave
    - [0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # Yaw
```

#### Ví dụ 3: Cấu hình 5-DOF (Không điều khiển Roll: Surge, Sway, Heave, Pitch, Yaw)
```yaml
dof_config:
  preset: "CUSTOM_5DOF"
  active_dofs:
    - "SURGE"
    - "SWAY"
    - "HEAVE"
    - "PITCH"
    - "YAW"
  # Nếu không nhập transform_matrix, ConfigLoader sẽ tự động suy ra T từ active_dofs!
```

> **Cơ chế nạp tự động của `ConfigLoader`**:
> - Nhận diện các tên DOF: `"SURGE"` / `"U"`, `"SWAY"` / `"V"`, `"HEAVE"` / `"W"`, `"ROLL"` / `"P"`, `"PITCH"` / `"Q"`, `"YAW"` / `"R"`.
> - Tự động chấp nhận các key ma trận trong YAML: `transform_matrix`, `transform_matrix_3dof_1`, `transform_matrix_4dof_1`, `transformation_matrix`, `projection_matrix`.
> - Nếu người dùng chỉ khai báo `active_dofs` mà bỏ qua ma trận, thư viện sẽ tự động sinh ma trận chiếu chuẩn trực giao qua hàm `DofTransformer::from_dof_names()`.

---

### 2.3. Cách 2: Khởi tạo động trong Runtime bằng mã C++

Nếu ứng dụng muốn thay đổi số bậc tự do động ngay trong khi chạy (chẳng hạn chuyển đổi linh hoạt giữa 3-DOF và 6-DOF khi robot chuyển chế độ hành trình):

```cpp
#include "nav_dynamics/dynamic_model.hpp"
#include "nav_dynamics/dof_transformer.hpp"

// Khởi tạo mô hình động lực học
nav_dynamics::DynamicModel model(vehicle_params);

// --- Tùy chọn A: Tạo từ danh sách tên DOF ---
auto dof_4 = nav_dynamics::DofTransformer::from_dof_names({"SURGE", "SWAY", "HEAVE", "YAW"});
model.set_dof_transformer(dof_4);

// --- Tùy chọn B: Tạo từ vector DofIndex enum ---
std::vector<nav_dynamics::DofIndex> active = {
    nav_dynamics::DofIndex::SURGE,
    nav_dynamics::DofIndex::HEAVE,
    nav_dynamics::DofIndex::YAW
};
nav_dynamics::DofTransformer dof_3(active);
model.set_dof_transformer(dof_3);

// --- Tùy chọn C: Tạo từ ma trận tùy chỉnh T (n x 6) ---
nav_dynamics::MatrixNd T(3, 6);
T.setZero();
T(0, 0) = 1.0; // u
T(1, 2) = 1.0; // w
T(2, 5) = 1.0; // r
nav_dynamics::DofTransformer custom_transformer(T, active);
model.set_dof_transformer(custom_transformer);
```

---

### 2.4. Cách 3: Thêm cấu hình Preset n-DOF cố định vào mã nguồn thư viện

Nếu bạn muốn đóng gói thêm một Preset chuẩn vào mã nguồn để dùng lại trên toàn hệ thống (ví dụ thêm `ROV_5DOF_CONFIG_1`):

#### Bước 1: Khai báo Preset trong [`include/nav_dynamics/dof_transformer.hpp`](file:///home/stevehoang/Navigation_System_Library/include/nav_dynamics/dof_transformer.hpp)
1. Thêm enum vào `enum class DofPreset`:
   ```cpp
   enum class DofPreset {
       ROV_6DOF_FULL,
       ROV_4DOF_CONFIG_1,
       ROV_3DOF_CONFIG_1,
       ROV_5DOF_CONFIG_1,  // <-- Thêm mới
   };
   ```
2. Thêm khai báo Factory helper trong `class DofTransformer`:
   ```cpp
   static DofTransformer make_rov_5dof();
   ```

#### Bước 2: Hiện thực logic trong [`src/dof_transformer.cpp`](file:///home/stevehoang/Navigation_System_Library/src/dof_transformer.cpp)
1. Trong hàm constructor `DofTransformer::DofTransformer(DofPreset preset)`, thêm nhánh `case`:
   ```cpp
   case DofPreset::ROV_5DOF_CONFIG_1:
       active_dofs_ = {DofIndex::SURGE, DofIndex::SWAY, DofIndex::HEAVE, 
                       DofIndex::PITCH, DofIndex::YAW};
       build_T_from_active_dofs();
       break;
   ```
2. Trong hàm `DofTransformer::from_preset`:
   ```cpp
   case DofPreset::ROV_5DOF_CONFIG_1:
       return make_rov_5dof();
   ```
3. Hiện thực hàm static factory:
   ```cpp
   DofTransformer DofTransformer::make_rov_5dof() {
       return DofTransformer(DofPreset::ROV_5DOF_CONFIG_1);
   }
   ```

#### Bước 3: Đăng ký nhận diện tên chuỗi trong [`src/config_loader.cpp`](file:///home/stevehoang/Navigation_System_Library/src/config_loader.cpp)
Trong hàm phụ trợ `parse_dof_transformer_from_node`:
```cpp
} else if (preset_str == "ROV_5DOF_CONFIG_1" || preset_str == "ROV_5DOF") {
    return DofTransformer::make_rov_5dof();
}
```

---

### 2.5. Bảng tổng hợp các hàm logic cốt lõi cho n-DOF

| Tên hàm | Thuộc lớp | Đầu vào | Đầu ra | Mục đích logic |
| :--- | :--- | :--- | :--- | :--- |
| `reduce_vector(v)` | `DofTransformer` | `const Vector6d& v` | `VectorNd` ($n \times 1$) | Thu giảm véc-tơ 6D xuống không gian $n$-DOF ($v_r = T v$) |
| `reduce_matrix(M)` | `DofTransformer` | `const Matrix6d& M` | `MatrixNd` ($n \times n$) | Thu giảm ma trận 6x6 xuống $n \times n$ ($M_r = T M T^T$) |
| `expand_vector(v_r)`| `DofTransformer` | `const VectorNd& v_r` | `Vector6d` | Mở rộng véc-tơ $n$-DOF thành véc-tơ 6D đầy đủ ($T^T v_r$) |
| `expand_matrix(M_r)`| `DofTransformer` | `const MatrixNd& M_r` | `Matrix6d` | Mở rộng ma trận $n \times n$ thành 6x6 ($T^T M_r T$) |
| `transform_thruster_allocation(B)` | `DofTransformer` | `const MatrixNd& B` ($6 \times k$) | `MatrixNd` ($n \times k$) | Chiếu ma trận phân bổ động cơ xuống $n$-DOF ($B_r = T B$) |
| `is_dof_active(dof)` | `DofTransformer` | `DofIndex dof` | `bool` | Kiểm tra xem 1 bậc tự do cụ thể có đang hoạt động |
| `is_orthogonal(tol)` | `DofTransformer` | `double tol = 1e-9` | `bool` | Kiểm định tính chuẩn trực giao: $T \cdot T^T = I_n$ |
| `project_to_active(v)` | `DofTransformer` | `const Vector6d& v` | `Vector6d` | Triệt tiêu các thành phần không thuộc $n$-DOF |
| `compute_forward_dynamics_reduced(...)` | `DynamicModel` | `state, tau_r, current` | `VectorNd` ($n \times 1$) | Giải phương trình động lực học trực tiếp trong không gian $n$-DOF |
| `apply_kinematic_constraints(state)` | `DynamicModel` | `KinematicState&` | `void` | Ép các vận tốc và góc quay không thuộc $n$-DOF về 0 |

---

## 3. HƯỚNG DẪN BUILD VÀ TÍCH HỢP THƯ VIỆN

### 3.1. Yêu cầu môi trường & Thư viện phụ thuộc
- **Hệ điều hành**: Linux (Ubuntu 20.04/22.04 khuyến nghị), macOS hoặc Windows.
- **Tiêu chuẩn C++**: C++17 trở lên (`-std=c++17`).
- **CMake**: Phiên bản tối thiểu $\ge 3.16$.
- **Thư viện phụ thuộc**:
  - `Eigen3` ($\ge 3.3$): Đại số tuyến tính, ma trận và vector.
  - `yaml-cpp`: Đọc và phân tích file cấu hình YAML.

Cài đặt phụ thuộc trên Ubuntu/Debian:
```bash
sudo apt update
sudo apt install -y cmake build-essential libeigen3-dev libyaml-cpp-dev
```

---

### 3.2. Build thư viện độc lập (Standalone Build)

Các bước build thư viện từ mã nguồn:
```bash
# 1. Đi vào thư mục gốc của repository
cd /path/to/Navigation_System_Library

# 2. Tạo thư mục build và cấu hình CMake
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# 3. Biên dịch toàn bộ thư viện, examples và tests
cmake --build build -j$(nproc)
```

Kết quả sau khi build:
- Thư viện tĩnh: `build/libnav_dynamics.a`
- File thực thi mô phỏng ví dụ: `build/examples/rov_simulation_example`
- Bộ test tự động: `build/tests/test_dof_transformer`, `build/tests/test_mass_matrix`, ...

Chạy chương trình mô phỏng ví dụ:
```bash
./build/examples/rov_simulation_example
```

Chạy bộ unit test:
```bash
ctest --test-dir build --output-on-failure
```

---

### 3.3. Tích hợp vào dự án khác bằng CMake (`add_subdirectory`)

Đây là phương thức phổ biến và tiện lợi nhất khi đưa thư viện vào dự án robot / navigation của bạn dưới dạng submodule hoặc thư mục con.

#### Cấu trúc thư mục dự án của bạn:
```text
my_robot_project/
├── CMakeLists.txt
├── config/
│   └── rov_params.yaml
├── src/
│   └── main.cpp
└── third_party/
    └── Navigation_System_Library/   <-- Đặt thư viện tại đây
```

#### Tệp `CMakeLists.txt` của dự án bạn:
```cmake
cmake_minimum_required(VERSION 3.16)
project(my_robot_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Tìm các thư viện phụ thuộc
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

# 2. Thêm thư mục Navigation_System_Library
add_subdirectory(third_party/Navigation_System_Library)

# 3. Tạo ứng dụng của bạn
add_executable(my_robot_node src/main.cpp)

# 4. Liên kết với target nav_dynamics
# nav_dynamics đã export sẵn PUBLIC include directories nên project bạn tự nhận header!
target_link_libraries(my_robot_node PRIVATE nav_dynamics)
```

Trong file mã nguồn `src/main.cpp`:
```cpp
#include <nav_dynamics/dynamic_model.hpp>
#include <nav_dynamics/config_loader.hpp>
#include <nav_dynamics/ros_adapter.hpp>
#include <iostream>

int main() {
    auto config = nav_dynamics::ConfigLoader::load_from_yaml("config/rov_params.yaml");
    nav_dynamics::DynamicModel rov(config.vehicle_params, config.dof_transformer);
    std::cout << "Robot initialized with " << rov.reduced_dim() << " DOFs.\n";
    return 0;
}
```

---

### 3.4. Tích hợp tự động qua CMake `FetchContent`

Nếu không muốn tải thủ công mã nguồn, bạn có thể tải tự động từ Git thông qua CMake:
```cmake
include(FetchContent)

FetchContent_Declare(
    nav_dynamics
    GIT_REPOSITORY https://github.com/YourOrganization/Navigation_System_Library.git
    GIT_TAG        main # Hoặc tag release ví dụ v1.1.0
)
FetchContent_MakeAvailable(nav_dynamics)

target_link_libraries(my_robot_node PRIVATE nav_dynamics)
```

---

### 3.5. Lưu ý quan trọng khi biên dịch

> [!NOTE]
> **Điểm lưu ý về `RovConfig::dof_config()`**:
> Trong cấu trúc `RovConfig`, trường `dof_transformer` là nguồn dữ liệu duy nhất (`single source of truth`). `dof_config()` được định nghĩa như một hàm phương thức getter trả về `DofConfig` tương thích ngược:
> ```cpp
> [[nodiscard]] DofConfig dof_config() const { return DofConfig(dof_transformer); }
> ```
> Do đó, khi truy cập `DofConfig` từ `RovConfig`, hãy gọi dưới dạng hàm: `auto cfg = rov_cfg.dof_config();`. Không gán trực tiếp biến thành viên này.

---

## 4. TRÍCH XUẤT DỮ LIỆU ĐẦU RA TỪ PHƯƠNG TRÌNH ĐỘNG LỰC HỌC & ĐỘNG HỌC

Thư viện cung cấp API rõ ràng, tường minh để lấy toàn bộ các tham số trung gian và tham số cuối cùng của cả hai mô hình động lực học và động học.

```
                           +----------------------------------------+
                           |  Thrusts u -> tau = B * u              |
                           +----------------------------------------+
                                              |
                                              v
+-----------------------+          +----------------------+          +----------------------+
| KinematicState        | -------> | DynamicModel         | -------> | Acceleration nu_dot  |
| (pos, rpy, quat, nu)  |          |                      |          | (6D & reduced n-DOF) |
+-----------------------+          +----------------------+          +----------------------+
                                              |                                  |
                                              v                                  v
                                   +----------------------+          +----------------------+
                                   | DynamicBreakdown     |          | Step RK4 / Euler     |
                                   | (tau, C*nu, D*nu, g) |          | New KinematicState   |
                                   +----------------------+          +----------------------+
```

---

### 4.1. Lấy gia tốc từ Động lực học thuận (Forward Dynamics)

Động lực học thuận giải phương trình chuyển động để tìm gia tốc trong hệ thân tàu $\dot{\nu}$:
$$\dot{\nu} = M^{-1} \Big( \tau - C(\nu)\nu - D(\nu_r)\nu_r - g(\eta) + M_A \dot{\nu}_c \Big)$$

#### A. Gia tốc trong không gian 6D (vẫn tuân thủ các ràng buộc n-DOF):
```cpp
nav_dynamics::KinematicState state;
// Gán vận tốc hiện tại: u=1.0 m/s, w=0.2 m/s, r=0.1 rad/s
state.nu << 1.0, 0.0, 0.2, 0.0, 0.0, 0.1;

// Lực và mô-men điều khiển đầu vào tau [X, Y, Z, K, M, N]^T (N và Nm)
nav_dynamics::Vector6d tau;
tau << 20.0, 0.0, 10.0, 0.0, 0.0, 5.0;

// Vận tốc dòng chảy môi trường (tùy chọn)
nav_dynamics::FluidCurrent current(nav_dynamics::Vector3d(0.1, 0.0, 0.0));

// Gọi hàm tính gia tốc 6D:
nav_dynamics::Vector6d nu_dot = rov_model.compute_forward_dynamics(state, tau, current);

// Đọc từng tham số gia tốc:
double u_dot = nu_dot(0); // Gia tốc tiến/lùi (Surge)    [m/s^2]
double v_dot = nu_dot(1); // Gia tốc dạt ngang (Sway)     [m/s^2]
double w_dot = nu_dot(2); // Gia tốc chìm/nổi (Heave)     [m/s^2]
double p_dot = nu_dot(3); // Gia tốc góc lắc ngang (Roll) [rad/s^2]
double q_dot = nu_dot(4); // Gia tốc góc chúi/ngóc (Pitch)[rad/s^2]
double r_dot = nu_dot(5); // Gia tốc góc quay trở (Yaw)   [rad/s^2]
```

#### B. Gia tốc trực tiếp trong không gian thu giảm $n$-DOF:
Nếu bạn đang điều khiển ở cấp controller $n$-DOF (ví dụ PID 3-DOF {Surge, Heave, Yaw}):
```cpp
// Lực điều khiển thu giảm tau_r (3x1)
nav_dynamics::VectorNd tau_r(3);
tau_r << 20.0, 10.0, 5.0; // [Surge Force, Heave Force, Yaw Torque]

nav_dynamics::VectorNd nu_dot_r = rov_model.compute_forward_dynamics_reduced(state, tau_r, current);

double surge_acc = nu_dot_r(0); // [m/s^2]
double heave_acc = nu_dot_r(1); // [m/s^2]
double yaw_acc   = nu_dot_r(2); // [rad/s^2]
```

---

### 4.2. Lấy phân tích chi tiết toàn bộ các thành phần lực (`DynamicBreakdown`)

Khi cần gỡ lỗi (debug), hiển thị đồ thị phân tích lực, hoặc áp dụng bộ quan sát trạng thái (Observer/EKF), bạn có thể lấy toàn bộ chi tiết phân tách từng lực trong phương trình Fossen thông qua struct `DynamicBreakdown`:

```cpp
nav_dynamics::DynamicBreakdown bd = rov_model.evaluate_breakdown(state, tau, current);

// 1. Lực điều khiển tác dụng tau
nav_dynamics::Vector6d f_tau = bd.control_wrench;

// 2. Lực Coriolis & Hướng tâm: C(nu)*nu
nav_dynamics::Vector6d f_coriolis = bd.coriolis_force;

// 3. Lực cản thủy động học tổng hợp: D(nu_r)*nu_r (Cản ma sát nhớt + Cản hình dáng)
nav_dynamics::Vector6d f_damping = bd.damping_force;

// 4. Lực và mô-men hồi phục thủy tĩnh: g(eta) (Trọng lực + Lực nổi)
nav_dynamics::Vector6d f_restoring = bd.restoring_force;

// 5. Lực hợp lực thuần tác dụng (Net Wrench): tau - C*nu - D*nu_r - g + f_current
nav_dynamics::Vector6d f_net = bd.net_wrench;

// 6. Gia tốc thu được: M^{-1} * net_wrench
nav_dynamics::Vector6d acc = bd.acceleration_6d;
```

---

### 4.3. Truy xuất trực tiếp các ma trận hệ thống (M, C, D, g, B)

Bạn có thể đọc trực tiếp các ma trận theo cả hai dạng 6x6 hoặc $n \times n$:

```cpp
// --- 1. MA TRẬN KHỐI LƯỢNG M ---
// Ma trận vật rắn M_RB (6x6)
const nav_dynamics::Matrix6d& M_RB = rov_model.mass_evaluator().M_RB();
// Ma trận khối lượng gia tăng M_A (6x6)
const nav_dynamics::Matrix6d& M_A = rov_model.mass_evaluator().M_A();
// Ma trận quán tính tổng M = M_RB + M_A (6x6)
const nav_dynamics::Matrix6d& M_total_6d = rov_model.mass_evaluator().M_total();
// Ma trận khối lượng thu giảm M_r = T * M * T^T (n x n)
nav_dynamics::MatrixNd M_r = rov_model.dof_transformer().reduce_matrix(M_total_6d);

// --- 2. MA TRẬN CORIOLIS C(nu) ---
nav_dynamics::Vector6d nu_r = current.compute_relative_velocity(state);
nav_dynamics::Matrix6d C_6d = rov_model.coriolis_evaluator().compute_coriolis_matrix(state.nu, nu_r);
nav_dynamics::MatrixNd C_r = rov_model.dof_transformer().reduce_matrix(C_6d);

// --- 3. MA TRẬN CẢN D(nu_r) ---
// Ma trận cản tuyến tính D_l (6x6)
const nav_dynamics::Matrix6d& D_l = rov_model.damping_evaluator().D_linear();
// Ma trận cản phi tuyến bậc hai D_q(nu_r) (6x6)
nav_dynamics::Matrix6d D_q = rov_model.damping_evaluator().compute_quadratic_damping(nu_r);
// Ma trận cản tổng hợp D(nu_r) = D_l + D_q (6x6)
nav_dynamics::Matrix6d D_total_6d = rov_model.damping_evaluator().compute_total_damping(nu_r);
// Ma trận cản thu giảm D_r = T * D * T^T (n x n)
nav_dynamics::MatrixNd D_r = rov_model.dof_transformer().reduce_matrix(D_total_6d);

// --- 4. VÉC-TƠ LỰC HỒI PHỤC g(eta) ---
nav_dynamics::Vector6d g_6d = rov_model.restoring_evaluator().compute_g_state(state);
nav_dynamics::VectorNd g_r = rov_model.dof_transformer().reduce_vector(g_6d);

// --- 5. MA TRẬN PHÂN BỔ ĐỘNG CƠ B ---
// Ma trận phân bổ 6xK (K là số lượng động cơ)
const nav_dynamics::MatrixNd& B_6k = rov_model.thruster_allocation().allocation_matrix();
// Ma trận phân bổ thu giảm n x K (B_r = T * B)
nav_dynamics::MatrixNd B_r = rov_model.dof_transformer().transform_thruster_allocation(B_6k);
```

---

### 4.4. Tính lực điều khiển yêu cầu từ Động lực học nghịch (Inverse Dynamics)

Cho trước gia tốc mong muốn $\dot{\nu}_{\text{des}}$, tính lực điều khiển $\tau$ cần thiết (dùng cho thuật toán Computed Torque Control / Feedback Linearization):
$$\tau = M \dot{\nu}_{\text{des}} + C(\nu)\nu + D(\nu_r)\nu_r + g(\eta) - M_A \dot{\nu}_c$$

```cpp
// Gia tốc mong muốn
nav_dynamics::Vector6d desired_nu_dot;
desired_nu_dot << 0.5, 0.0, -0.1, 0.0, 0.0, 0.05;

// Tính lực và mô-men 6D yêu cầu:
nav_dynamics::Vector6d required_tau = rov_model.compute_inverse_dynamics(state, desired_nu_dot, current);

// Hoặc tính trực tiếp trong không gian n-DOF:
nav_dynamics::VectorNd desired_nu_dot_r(3);
desired_nu_dot_r << 0.5, -0.1, 0.05;
nav_dynamics::VectorNd required_tau_r = rov_model.compute_inverse_dynamics_reduced(state, desired_nu_dot_r, current);
```

---

### 4.5. Lấy dữ liệu tư thế và vị trí từ Phương trình Động học (Kinematics)

Thư viện tích hợp phương trình vi phân động học $\dot{p}_{ned} = R_b^n \nu_{linear}$ và cập nhật quaternion $\dot{q} = \frac{1}{2} q \otimes \omega_{body}$ qua hai thuật toán tích phân:

```cpp
double dt = 0.05; // Bước thời gian mô phỏng (ví dụ: 20 Hz = 0.05s)

// Cách 1: Tích phân số Runge-Kutta bậc 4 (Độ chính xác cao, khuyến nghị)
rov_model.step_rk4(state, tau, dt, current);

// Cách 2: Tích phân số Euler (Nhanh, phù hợp vi điều khiển nhúng)
// rov_model.step_euler(state, tau, dt, current);
```

Sau khi gọi hàm bước nhảy thời gian, đối tượng `state` (`KinematicState`) chứa toàn bộ các thông số động học mới:

```cpp
// 1. VỊ TRÍ TRONG HỆ NED [m]
double pos_north = state.pos_ned.x(); // Tọa độ Bắc (X_ned)
double pos_east  = state.pos_ned.y(); // Tọa độ Đông (Y_ned)
double depth     = state.pos_ned.z(); // Độ sâu (Z_ned, giá trị dương hướng xuống)

// 2. GÓC ĐỊNH HƯỚNG EULER (ZYX Convention) [rad]
double roll  = state.euler_rpy.x(); // Góc lắc ngang (phi)
double pitch = state.euler_rpy.y(); // Góc chúi/ngóc (theta)
double yaw   = state.euler_rpy.z(); // Góc hướng la bàn (psi)

// 3. QUATERNION ĐỊNH HƯỚNG (Bảo toàn, tránh suy biến Gimbal Lock)
double qw = state.orientation.w();
double qx = state.orientation.x();
double qy = state.orientation.y();
double qz = state.orientation.z();

// 4. VẬN TỐC TRONG HỆ THÂN TÀU (Body-fixed) [m/s và rad/s]
double u = state.nu(0); // Vận tốc tiến/lùi (Surge)
double v = state.nu(1); // Vận tốc dạt ngang (Sway)
double w = state.nu(2); // Vận tốc chìm/nổi (Heave)
double p = state.nu(3); // Vận tốc góc roll
double q = state.nu(4); // Vận tốc góc pitch
double r = state.nu(5); // Vận tốc góc yaw

// 5. GIA TỐC TRONG HỆ THÂN TÀU [m/s^2 và rad/s^2]
nav_dynamics::Vector6d acc_body = state.nu_dot;

// 6. MA TRẬN QUAY VÀ BIẾN ĐỔI HÌNH HỌC
nav_dynamics::Matrix3d R_nb = state.R_nb(); // Quay từ Thân tàu -> NED
nav_dynamics::Matrix3d R_bn = state.R_bn(); // Quay từ NED -> Thân tàu

// Vận tốc tịnh tiến trong hệ quy chiếu NED:
nav_dynamics::Vector3d vel_ned = state.R_nb() * state.nu.head<3>();

// 7. MA TRẬN JACOBIAN ĐỘNG HỌC TOÀN PHẦN J(eta) (6x6)
nav_dynamics::Matrix6d J = state.J_full();
```

---

### 4.6. Trích xuất lực đẩy động cơ và tín hiệu PWM

Lớp `ThrusterAllocation` chuyển đổi giữa lực tổng quát $\tau$ và các động cơ vật lý:

```cpp
// 1. Phân bổ lực ngược: Từ Wrench mong muốn tau (6x1) -> Lực từng động cơ (N)
// Thuật toán: u = B^{\dagger} * tau (dùng SVD giả nghịch đảo Moore-Penrose)
nav_dynamics::VectorNd thrusts = rov_model.thruster_allocation().inverse_allocation(tau);

for (size_t i = 0; i < rov_model.thruster_allocation().num_thrusters(); ++i) {
    std::cout << "Động cơ " << i << " (" << rov_model.thruster_allocation().thruster(i).name 
              << "): Lực = " << thrusts(i) << " [N]\n";
}

// 2. Chuyển đổi lực đẩy sang tín hiệu PWM ArduSub (Microseconds [us])
std::vector<double> pwms = rov_model.thruster_allocation().thrusts_to_pwm(thrusts);
for (size_t i = 0; i < pwms.size(); ++i) {
    std::cout << "PWM Motor " << i << " = " << pwms[i] << " us\n";
}

// 3. Phân bổ lực thuận: Từ lực động cơ -> Wrench 6D tác dụng lên thân tàu: tau = B * u
nav_dynamics::Vector6d resulting_tau = rov_model.thruster_allocation().forward_allocation(thrusts);
```

---

### 4.7. Chuẩn hóa dữ liệu sang kiểu ROS POD (Zero ROS Dependency)

Để giao tiếp trơn tru với ROS/ROS2 hoặc gửi qua mạng (UDP/WebSocket/Protobuf) mà không cần cài đặt ROS, hãy dùng `RosAdapter`:

```cpp
#include "nav_dynamics/ros_adapter.hpp"

// 1. Xuất thông điệp Odometry (tương đương nav_msgs/Odometry)
nav_dynamics::OdometryPOD odom = nav_dynamics::RosAdapter::to_odometry_pod(state);
// odom.pose.position.x/y/z
// odom.pose.orientation.x/y/z/w
// odom.twist.linear.x/y/z
// odom.twist.angular.x/y/z

// 2. Xuất thông điệp Pose (tương đương geometry_msgs/Pose)
nav_dynamics::PosePOD pose = nav_dynamics::RosAdapter::to_pose_pod(state);

// 3. Xuất thông điệp Twist (tương đương geometry_msgs/Twist)
nav_dynamics::TwistPOD twist = nav_dynamics::RosAdapter::to_twist_pod(state.nu);

// 4. Xuất thông điệp Wrench (tương đương geometry_msgs/Wrench)
nav_dynamics::WrenchPOD wrench = nav_dynamics::RosAdapter::to_wrench_pod(tau);

// 5. Xuất thông điệp Accel (tương đương geometry_msgs/Accel)
nav_dynamics::AccelPOD accel = nav_dynamics::RosAdapter::to_accel_pod(state.nu_dot);
```

---

## 5. VÍ DỤ HOÀN CHỈNH TÍCH HỢP END-TO-END

Dưới đây là một chương trình C++ hoàn chỉnh minh họa việc nạp cấu hình, cấu hình $n$-DOF, thực thi vòng lặp mô phỏng, và lấy toàn bộ dữ liệu đầu ra:

```cpp
#include <iostream>
#include <iomanip>
#include "nav_dynamics/dynamic_model.hpp"
#include "nav_dynamics/config_loader.hpp"
#include "nav_dynamics/ros_adapter.hpp"

int main() {
    // 1. Nạp cấu hình xe từ YAML
    std::string yaml_path = "config/rov_params.yaml";
    nav_dynamics::RovConfig cfg = nav_dynamics::ConfigLoader::load_from_yaml(yaml_path);

    std::cout << ">>> Nạp thành công cấu hình: " << cfg.vehicle_name << std::endl;
    std::cout << ">>> Số bậc tự do hoạt động: " << cfg.dof_transformer.reduced_dim() << std::endl;
    for (const auto& dof_name : cfg.dof_transformer.active_dof_names()) {
        std::cout << "    * " << dof_name << std::endl;
    }

    // 2. Khởi tạo mô hình động lực học
    nav_dynamics::DynamicModel model(cfg.vehicle_params, cfg.dof_transformer, cfg.thruster_allocation);

    // 3. Khởi tạo trạng thái ban đầu và dòng chảy đại dương
    nav_dynamics::KinematicState state;
    state.pos_ned = nav_dynamics::Vector3d(0.0, 0.0, 1.0); // Độ sâu ban đầu 1.0 m
    nav_dynamics::FluidCurrent ocean_current(cfg.ned_env.nominal_ocean_current);

    // 4. Thiết lập lệnh điều khiển: Lực động cơ đẩy [TL, TR, TV] (N)
    nav_dynamics::VectorNd thrust_cmds(3);
    thrust_cmds << 10.0, 10.0, 5.0; // 10N tiến, 5N đẩy chìm

    // Chuyển đổi lực động cơ sang wrench 6D: tau = B * u
    nav_dynamics::Vector6d tau = cfg.thruster_allocation.forward_allocation(thrust_cmds);

    // 5. Vòng lặp mô phỏng thời gian thực (ví dụ 10 bước với dt = 0.05s)
    double dt = 0.05;
    std::cout << "\n--- BẮT ĐẦU MÔ PHỎNG ---\n";
    std::cout << std::fixed << std::setprecision(4);

    for (int step = 0; step < 10; ++step) {
        double current_time = step * dt;

        // Trích xuất các thành phần lực chi tiết tại bước hiện tại
        nav_dynamics::DynamicBreakdown bd = model.evaluate_breakdown(state, tau, ocean_current);

        // Tiến hành 1 bước tích phân RK4
        model.step_rk4(state, tau, dt, ocean_current);

        // Đọc dữ liệu đầu ra từ KinematicState
        std::cout << "t = " << current_time << " s | "
                  << "Vị trí [X, Y, Z]: (" 
                  << state.pos_ned.x() << ", " 
                  << state.pos_ned.y() << ", " 
                  << state.pos_ned.z() << ") m | "
                  << "Vận tốc [u, w, r]: (" 
                  << state.nu(0) << " m/s, " 
                  << state.nu(2) << " m/s, " 
                  << state.nu(5) << " rad/s) | "
                  << "Gia tốc u_dot: " << bd.acceleration_6d(0) << " m/s^2 | "
                  << "Lực cản D_u: " << bd.damping_force(0) << " N\n";
    }

    // 6. Trích xuất dữ liệu ra chuẩn ROS POD
    nav_dynamics::OdometryPOD odom = nav_dynamics::RosAdapter::to_odometry_pod(state);
    std::cout << "\n>>> Dữ liệu ROS Odometry cuối cùng:\n";
    std::cout << "    Position : [" << odom.pose.position.x << ", " 
                                     << odom.pose.position.y << ", " 
                                     << odom.pose.position.z << "]\n";
    std::cout << "    LinearVel: [" << odom.twist.linear.x << ", " 
                                     << odom.twist.linear.y << ", " 
                                     << odom.twist.linear.z << "]\n";

    return 0;
}
```
