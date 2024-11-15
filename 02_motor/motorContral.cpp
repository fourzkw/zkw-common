#include <iostream>
#include <vector>

using namespace std;

class motor {
private:
  bool isOn;
  int speed;
  int dir;

public:
  motor();
  ~motor();
  bool changeState();
  bool setSpeed(int s);
  bool setDir(int d);
  bool getState();
  int getSpeed();
  int getDir();
};

motor::motor() {
  isOn = 0;
  speed = 0;
  dir = 0;
  printf("创建成功\n");
}

motor::~motor() {}

bool motor::changeState() {
  isOn = !isOn;
  return true;
}

bool motor::setSpeed(int s) {
  if (isOn) {
    speed = s;
    return true;
  }
  return false;
}

bool motor::setDir(int d) {
  if (isOn) {
    dir = d;
    return true;
  } else
    return false;
}

bool motor::getState() { return isOn; }

int motor::getSpeed() { return speed; }
int motor::getDir() { return dir; }

vector<motor> motors;
int curNum = -1;

void PrintPanel() {
  printf("------------------------------------------\n");
  printf("请输入数字进行选择:\n");
  printf("0.选择电机\n");
  printf("1.设置一个新电机\n");
  printf("2.开关电机\n");
  printf("3.修改电机速度\n");
  printf("4.修改电机方向\n");
  printf("5.查看电机参数\n");
  printf("6.删除当前电机\n");
  printf("-1.退出程序\n");
  printf("当前电机为：%d\n", curNum);
  printf("------------------------------------------\n");
  return;
}

void PrintMotorPanel() {
  printf("------------------------------------------\n");
  if (motors.size() > 0)
    printf("当前可选电机：%d ~ %d\n", 0, motors.size() - 1);
  else
    printf("当前无可选电机\n");
  printf("输入数字选择，输入-1返回\n");
  printf("------------------------------------------\n");
  return;
}

int main() {
  while (1) {
    int n; // 主选择
    PrintPanel();
    cin >> n;
    if (n == 0) {
      int s; // 选择电机
      PrintMotorPanel();
      while (cin >> s) {
        if (s == -1)
          break;
        else if (s >= motors.size()) {
          printf("无此电机，请重新选择\n");
        } else {
          curNum = s;
          break;
        }
      }
    } else if (n == 1) {
      motor m;
      motors.push_back(m);
    } else if (n == 2) {
      if (curNum == -1) {
        printf("请先选择电机\n");
        continue;
      }
      motors[curNum].changeState();
    } else if (n == 3) { //修改电机速度
      if (curNum == -1) {
        printf("请先选择电机\n");
        continue;
      }
      if (motors[curNum].getState() == false) {
        printf("请先启动电机\n");
        continue;
      }
      int s;
      printf("NewSpeed:");
      cin >> s;
      if (s != 0) {
        motors[curNum].setSpeed(abs(s));
        motors[curNum].setDir(s/abs(s));
      } else if (s == 0) {
        motors[curNum].setSpeed(0);
        motors[curNum].setDir(0);
      }
      printf("设置成功\n");
    } else if (n == 4) {
      if (curNum == -1) {
        printf("请先选择电机\n");
        continue;
      }
      if (motors[curNum].getState() == false) {
        printf("请先启动电机\n");
        continue;
      }
      int s;
      printf("NewDirection:");
      while (1) {
        cin >> s;
        if (s == 0) {
          motors[curNum].setSpeed(0);
          motors[curNum].setDir(0);
          printf("设置成功\n");
          break;
        } else if (s == 1 || s == -1) {
          motors[curNum].setDir(s);
          printf("设置成功\n");
          break;
        } else {
          printf("设置失败，请输入-1或0或1\n");
        }
      }
    } else if (n == 5) {
      printf("State: %d\n", motors[curNum].getState());
      printf("Speed: %d\n", motors[curNum].getSpeed());
      printf("Direction: %d\n", motors[curNum].getDir());
    } else if (n == 6) {
      if (curNum == -1) {
        printf("当前没有选择电机\n");
        continue;
      }
      for (int i = curNum; i < motors.size() - 1; i++)
        motors[i] = motors[i + 1];
      motors.pop_back();
      curNum = -1;
      printf("删除成功\n");
    } else if (n == -1)
      break;
    else {
      printf("输入无效，请重新输入\n");
    }
  }
  return 0;
}