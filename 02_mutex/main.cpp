#include <iostream>
#include <mutex>
#include <thread>

using namespace std;

mutex mtx;

string inputStr;
bool isStart = true;

void releaseHydrogen(int &Hnum, int &Cnum) {
  std::lock_guard<std::mutex> lk(mtx);
  while (isStart || (Hnum >= 4 && Cnum >= 1)) {
    if (Hnum >= 4 && Cnum >= 1) {
      Hnum -= 4;
      Cnum--;
      cout << "HHHHC";
    }
  }
}

void releaseCarbon(int &Hnum, int &Cnum) {
  std::lock_guard<std::mutex> lk(mtx);
  while (isStart || (Hnum >= 4 && Cnum >= 1)) {
    if (Hnum >= 4 && Cnum >= 1) {
      Hnum -= 4;
      Cnum--;
      cout << "CHHHH";
    }
  }
}

int main() {
  int H_number = 0;
  int C_number = 0;
  cin >> inputStr;
  thread Hthread(releaseHydrogen, ref(H_number), ref(C_number));
  thread Cthread(releaseCarbon, ref(H_number), ref(C_number));
  isStart = true;
  for (int i = 0; i < inputStr.size(); i++) {
    if (inputStr[i] == 'C')
      C_number++;
    else if (inputStr[i] == 'H')
      H_number++;
  }

  // cout << endl;
  // cout << "C " << C_number << "   H " << H_number << endl;
  isStart = false;
  Hthread.join();
  Cthread.join();
  return 0;
}