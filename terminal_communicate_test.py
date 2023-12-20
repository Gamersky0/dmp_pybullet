import subprocess
import time
import select
import msvcrt

def is_data_available(f):
    return msvcrt.kbhit()

platform = "windows"

# 运行C++程序并与其通信
if platform.lower() == "linux":
    cpp_process = subprocess.Popen(["./cpp_program"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
else:
    cpp_process = subprocess.Popen(["D:\work\Code\VS2022_Project\DMP_Communication_Test_12111643\single_dmp_test.exe"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)

count = 0
# 输入起始和终止点

message_to_a = "Initialization:0.0,0.0,0.0,1.0,1.0,1.0"
cpp_process.stdin.write(message_to_a + "\n")    # 写入消息
cpp_process.stdin.flush()    # 刷新输入流  

while True:
    count += 1

    # 从A接收消息
    response_from_a = cpp_process.stdout.readline().strip() # 阻塞读
    print("B:", response_from_a)

    # 向A发送消息
    message_to_a = "continue"
    cpp_process.stdin.write(message_to_a + "\n")    # 写入消息
    cpp_process.stdin.flush()    # 刷新输入流

    if count == 2000:
        message_to_a = "exit"
        cpp_process.stdin.write(message_to_a + "\n")    # 写入消息
        cpp_process.stdin.flush()    # 刷新输入流
        break

    print("chkpnt end")
    # time.sleep(0.01)

# 关闭A程序的输入和输出流，并等待其结束
cpp_process.stdin.close()
cpp_process.stdout.close()
cpp_process.wait()