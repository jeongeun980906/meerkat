# meerkat

============
## carduino

1. node serial_node2.py

``` 
rosrun carduino serial_node2.py
```
    실행시 자동으로 /dev/ttyACM1 아두이노로 새로운 로스 시리얼 노드가 만들어진다. 
    rosrun rosserial_python serial_node.py /dev/ttyACM0와 함께 사용 가능하다. 

2. imu.ino
    MPU 6050의 IMU정보를 아두이노로 받아서 로스로 publish 해준다. 
    String type으로 전달되며 토픽이름은 /chatter

String data = " A " + AX + " B " + AY + " C " + AZ + " D " + GX + " E " + GY + " F " + GZ + " G " 형식이다. 

여기서 가속도가 AX,AY,AZ 자이로가 GX,GY,GZ인데 둘이 반댄거같다. ~~뒤에 칼만 필터에서 바꿔서 씀~~

3. servo.ino
차량의 바퀴를 제어한다. /servoang과 /servovel 두가지 토픽을 subscribe 해준다. 모두 타입은 std_msgs/Int16이다.

servovel의 경우 91이 멈춤. 86이 천천히 전진이다. 숫자를 더 낮추면 빨리간다. 

servoang의 경우 90도를 기준으로 steering wheel을 꺾는다. 

4. dcsv.ino
후진을 위한 코드. L293D라는 드라이버를 함께 사용한다. 

위와 같지만 120정도 주어야지 천천히 후진한다. 

============
## meerkat_camera

**수식은 [KAsimov WiKi](https://kasimov.korea.ac.kr/dokuwiki/doku.php/activity/member/2020/meerkat) 참조**

1. camera_1.py camera_2.py
``` 
rosrun meerkat_camera camera_2.py
```
리얼센스 435i로 차량을 인식하여 world frame을 기준으로 로봇의 현재 x,y,yaw를 /position_kf 로 publish 한다.

1버전은 수식을 그래도 쓴 것인데 무언가 안맞아서 2버전을 만들어서 맞을 때 까지 식을 수정하였다. 

차량 인식은 노란색과 녹색 표시기를 컬러 필터로 필터링 한 뒤 컨투어를 잡고 이의 bounding box를 만드는 형식이다. 
차량의 앞(녹색) 차량의 뒤(노란색)을 각각 인식하여 중간 값을 x,y로 지정하고 yaw방향까지 인식해주었다.

2. kalman_filter2.py
``` 
rosrun meerkat_camera kalman_filter2.py
```

순수하게 칼만 필터만 적용하여 필터 결과를 보기 위한 코드이다. velocity 또한 볼 수 있으므로 속도 측정하고 위치 정확도를 보는데에 활용하였다. 

수식은 위키에 적어 놓았다. 미리 정해놓은 움직임에 따라 움직이고 process update하고 측정하고 measurement update하는 프로세스를 반복한다. 


3. realworld_rl.py
``` 
rosrun meerkat_camera realworld_rl.py
```
시뮬레이터로 학습된 네비게이션 강화학습 알고리즘을 적용한 코드이다. 

칼만 필터가 적용된 x,y,yaw 추정 치를 input 데이터로 활용한다. 

~~멈추는 기능이 없으니 다 도착하면 그 순간에 수동으로 멈춰줘야한다. 코드 수정 필요~~

============
## m2wr_description

~~무언가 중간에 철자가 자주 틀렸다...~~
``` 
roslaunch m2wr_description spawn2.py
```

시뮬레이터상의 학습을 위한 가제보 런치 파일이다. 
프로젝트의 차량에 맞추어 제작하였으며 라이다 센서가 부착되어있다.

/odom이라는 토픽으로 현재 위치의 추정치를 볼 수 있고 /cmd_vel로 움직일 수 있다. 

============
## meerkat_rl

**수식은 [KAsimov WiKi](https://kasimov.korea.ac.kr/dokuwiki/doku.php/activity/member/2020/meerkat) 참조**

1. environment4.py, per-d3qn.py
``` 
rosrun meerkat_rl per-d3qn.py
```

실행전에 가제보를 키는 런치파일을 먼저 실행 하여야한다. 

여러가지 버전이 있는데 시뮬레이터 상에서는 환경4가 가장 잘되었다. 
input은 스캔 데이터와 로봇 좌표계에 대한 목표 위치의 좌표이다. action은 steerning angle이며 알고리즘은 PER-D3QN을 사용하였다.  
여기서 action으로 주는 속도는, 칼만 필터 추정이 아닌 속도 측정 실험에 의한 결과 값이고 노이즈 또한 추가 하지 않았다. 
자세한 수식과 결과는 위의 위키에 적혀있다. 

##########아직 수정 중###########

## project_practice_1

원래는 숫자를 인식하여 정해진 호실의 방에 들어가는 것을 하기 위하여 숫자 인식 기능을 만들어 놓았다. 

데이터셋은 MNIST 손글씨 데이터 셋으로 학습하였으며 그 모델의 가중치들을 불러와서 인식한다. 이를 위하여 숫자의 컨투어를 잡고 각각의 숫자를 따서 28by28로 변한한 뒤, 각각의 자릿수 별로 인식하여 다시 합친다. 

service로 숫자를 지정하면 0/1 맞는지 틀린지, 그리고 0,1,2로 가운데에 있는지, 왼쪽에 있는지, 오른쪽에 있는지를 판단한다. 

**preprocessing2.py**가 이 코드이고 **test_client.py**로 확인해 볼 수 있다. 

**preprocessing1.py**은 서비스 기능 없이 똑같이 숫자를 인식하여 result를 프린트만 한다. 

그 외의 벽을 따라가는 알고리즘 등 폐기 처분한 코드들이 있다. 


