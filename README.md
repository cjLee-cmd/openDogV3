# OpenDog V3 Quadruped Robot

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Raspberry Pi](https://img.shields.io/badge/platform-Raspberry%20Pi-red.svg)](https://www.raspberrypi.org/)

OpenDog V3는 Arduino C++ 코드를 Python으로 완전 변환한 4족 보행 로봇입니다. 실시간 역기구학 제어, IMU 기반 안정성 제어, 무선 원격 제어 기능을 제공합니다.

## 🎯 주요 특징

- **실시간 제어**: 100Hz 제어 루프로 정밀한 모션 제어
- **모듈식 아키텍처**: 독립적이고 재사용 가능한 구성 요소
- **포괄적인 테스트**: 자동화된 테스트 스위트 및 하드웨어 진단
- **무선 통신**: nRF24L01 기반 원격 제어
- **안정성 제어**: IMU 센서를 활용한 동적 균형 유지
- **다양한 보행 패턴**: 트롯, 워크, 바운드 보행 지원

## 📁 프로젝트 구조

```
openDogV3/
├── Code/
│   ├── openDogV3_experimental_stability/    # 메인 Python 구현
│   │   ├── opendog_v3_controller.py         # 메인 로봇 컨트롤러
│   │   ├── kinematics.py                    # 역기구학 모듈
│   │   ├── odrive_interface.py              # ODrive 모터 제어
│   │   ├── imu_interface.py                 # IMU 센서 인터페이스
│   │   ├── input_processing.py              # 입력 처리 및 보행 패턴
│   │   ├── remote_controller.py             # 무선 원격 제어기
│   │   ├── config.py                        # 설정 관리
│   │   ├── test_suite.py                    # 종합 테스트 스위트
│   │   └── hardware_setup.py                # 하드웨어 설정 유틸리티
│   ├── openDogV3/                          # 원본 Arduino 코드
│   └── Remote/                             # 원격 제어기 Arduino 코드
├── CAD/                                    # 3D 모델 및 설계 파일
├── MANUAL.md                               # 상세 프로그램 매뉴얼 (한국어)
├── MANUAL_EN.md                            # 상세 프로그램 매뉴얼 (English)
├── BOM.ods                                 # 부품 명세서
└── README.md                               # 이 파일
```

## 🛠️ 하드웨어 요구사항

### 메인 시스템
- **Raspberry Pi 4 Model B** (4GB+ 권장)
- **ODrive 모터 컨트롤러** v3.6 (2개)
- **브러시리스 DC 모터** 8개 (각 다리당 2개)
- **IMU 센서** (MPU6050 또는 호환)
- **nRF24L01 무선 모듈**

### 원격 제어기
- **Raspberry Pi Zero W** 또는 Pi 4
- **20x4 LCD 디스플레이** (I2C)
- **조이스틱 및 버튼** (아날로그/디지털 입력)
- **nRF24L01 무선 모듈**

### 전원 시스템
- **24V 리튬 폴리머 배터리** (6S, 5000mAh+)
- **전원 분배 보드**
- **BMS (배터리 관리 시스템)**

## 💻 소프트웨어 요구사항

- **Raspberry Pi OS** (Bullseye 이상)
- **Python 3.8+**
- **필수 패키지**:
  ```
  numpy>=1.21.0
  odrive>=0.6.0
  pyserial>=3.5
  RPi.GPIO>=0.7.1
  smbus2>=0.4.1
  spidev>=3.5
  circuitpython-nrf24l01>=1.2.3
  ```

## 🚀 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone <repository-url> opendog_v3
cd opendog_v3

# 가상환경 생성 및 활성화
python3 -m venv opendog_env
source opendog_env/bin/activate

# 의존성 설치
pip install -r requirements.txt

# 권한 설정
sudo usermod -a -G gpio,spi,i2c,dialout $USER
```

### 2. 하드웨어 설정

```bash
cd Code/openDogV3_experimental_stability

# 하드웨어 진단 및 설정
python3 hardware_setup.py

# IMU 캘리브레이션
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"
```

### 3. 로봇 실행

```bash
# 메인 컨트롤러 시작
python3 opendog_v3_controller.py

# 원격 제어기 시작 (별도 터미널)
python3 remote_controller.py
```

## 🧪 테스트

### 전체 테스트 실행
```bash
python3 test_suite.py
```

### 개별 구성 요소 테스트
```bash
# 역기구학 테스트
python3 -c "from test_suite import TestSuite; TestSuite().test_kinematics()"

# 모터 제어 테스트
python3 -c "from test_suite import TestSuite; TestSuite().test_odrive_interface()"

# IMU 테스트
python3 -c "from test_suite import TestSuite; TestSuite().test_imu_interface()"
```

## 📊 성능 벤치마크

시스템은 다음 성능 목표를 달성합니다:

- **제어 주기**: 10ms (100Hz)
- **역기구학 계산**: <1ms
- **무선 통신 지연**: <5ms
- **IMU 업데이트**: 1kHz
- **배터리 수명**: 30-60분 (사용량에 따라)

## 🎮 제어 모드

### 모드 1: 수동 제어
- 개별 다리 위치 직접 제어
- 테스트 및 캘리브레이션에 적합

### 모드 2: 보행 제어
- 자동 보행 패턴 (트롯, 워크, 바운드)
- 방향 및 속도 명령으로 제어

### 모드 3: 자세 제어
- 정적 자세 유지
- 몸체 롤, 피치, 요 제어

### 모드 4: 균형 제어
- IMU 피드백 기반 동적 균형
- PID 제어 알고리즘

## 🔧 설정

주요 설정은 `config.py`에서 관리됩니다:

```python
# 제어 매개변수
CONTROL_FREQUENCY = 100  # Hz
SAFETY_TIMEOUT = 1.0     # seconds

# 기구학 매개변수
LEG_LENGTH_1 = 110  # mm
LEG_LENGTH_2 = 120  # mm
BODY_WIDTH = 160    # mm
BODY_LENGTH = 280   # mm

# 안전 제한
MAX_MOTOR_CURRENT = 20  # A
MAX_MOTOR_VELOCITY = 50 # rad/s
```

## 📖 상세 문서

완전한 설치, 설정, 운영 가이드는 다음 매뉴얼을 참조하세요:

- [📚 프로그램 매뉴얼 (한국어)](MANUAL.md) - 포괄적인 설치 및 운영 가이드
- [📚 Program Manual (English)](MANUAL_EN.md) - Comprehensive installation and operation guide

## 🐛 문제 해결

### 일반적인 문제

#### ODrive 연결 실패
```bash
# 시리얼 포트 확인
ls -la /dev/ttyACM*

# 권한 추가
sudo usermod -a -G dialout $USER
```

#### IMU 읽기 실패
```bash
# I2C 장치 스캔
i2cdetect -y 1

# IMU 재초기화
python3 hardware_setup.py
```

#### 성능 문제
```bash
# 시스템 리소스 확인
htop
free -h

# 프로세스 우선순위 조정
sudo renice -10 $(pgrep python3)
```

자세한 문제 해결 가이드는 [매뉴얼](MANUAL.md#문제-해결)을 참조하세요.

## 🔄 개발 로드맵

- [x] Arduino C++ 코드를 Python으로 변환
- [x] 모듈식 아키텍처 구현
- [x] 원격 제어기 시스템
- [x] 종합 테스트 스위트
- [x] 하드웨어 설정 유틸리티
- [ ] 웹 기반 제어 인터페이스
- [ ] 컴퓨터 비전 통합
- [ ] 머신러닝 기반 보행 최적화
- [ ] ROS 통합

## 🤝 기여하기

기여를 환영합니다! 다음과 같은 방식으로 참여할 수 있습니다:

1. **버그 리포트**: Issues 탭에서 버그를 보고해주세요
2. **기능 요청**: 새로운 기능에 대한 아이디어를 제안해주세요
3. **코드 기여**: Pull Request를 통해 코드를 기여해주세요
4. **문서 개선**: 문서의 개선사항을 제안해주세요

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 있습니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 🙏 감사의 말

- **원본 OpenDog 프로젝트** - 기본 하드웨어 설계 및 아이디어
- **XRobots** - 영감과 커뮤니티 지원: https://www.youtube.com/playlist?list=PLpwJoq86vov8uTgd8_WNgBHFpDYemO-OJ
- **ODrive 팀** - 뛰어난 모터 컨트롤러
- **Raspberry Pi 재단** - 강력한 SBC 플랫폼

---

## 원본 Arduino 버전 정보

### 엔코더 설정
AS5047 엔코더를 절대 위치 모드로 사용합니다. ODrive 문서를 참조하여 엔코더 매개변수를 구성하고 오프셋 캘리브레이션을 실행하세요: https://docs.odriverobotics.com/v/latest/encoders.html

ODrive 도구 내에서 vel_limit 및 vel_limit_tolerance를 math.inf로 설정하여 특정 상황에서 모터가 비활성화되는 것을 방지합니다. (ODrive 도구에서 먼저 'import math'를 실행해야 합니다)

### 원본 메뉴 옵션 (Arduino 버전)
0) 기본값 / 전원 켜기 시 기본값
1) 모터를 폐쇄 루프 제어로 설정
2) 다리를 바깥쪽으로 이동하여 스탠드 스터럽을 1-2mm 정도 클리어
3) 어깨와 무릎 관절이 모두 45도가 되도록 다리 이동 (CAD에 표시된 기본 위치)
4) 모터 위치, 속도 및 적분기 게인 증가
5) 6축 변환 및 회전을 위한 역기구학 데모 모드
6) 보행 모드 (5와 동일한 다리 위치)
10) 발을 스탠드 스터럽에 놓이도록 다시 위치시킴

### 하드웨어 제작 정보

**원격 제어기**: 'reverse' 스위치가 있어 4개 축을 역방향으로 만들어 로봇이 뒤로 걸을 수 있습니다. 이것은 로봇의 기구학이 아닌 원격 제어기에서 처리됩니다. 또한 로봇이 작동하려면 켜져 있어야 하는 모터 활성화 스위치가 있습니다.

**발**: 실리콘 고무 발을 위한 발 몰드 CAD가 포함되어 있습니다. 25A Shore 경도 플래티넘 경화 실리콘을 안료와 함께 사용했습니다. 탄소 섬유 발 튜브가 하부 다리와 발 인서트에 접착되어 회전을 방지한다는 점에 유의하세요.

**3D 프린팅**: 부품은 모두 PLA로 프린팅됩니다. 큰 부품은 0.3mm 레이어 높이에서 3개 둘레로 약 15% 채움률입니다. 사이클로이달 드라이브 내부와 같은 작은 부품은 4개 둘레와 최대 30-40% 채움률입니다.

---

*OpenDog V3 - Python으로 구현된 차세대 4족 보행 로봇*
