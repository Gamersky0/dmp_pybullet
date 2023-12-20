import subprocess

platform = "windows"

# 运行C++程序并与其通信
if platform.lower() == "linux":
    cpp_process = subprocess.Popen(["./cpp_program"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)
else:
    cpp_process = subprocess.Popen(["D:\work\Code\VS2022_Project\DMP_Communication_test\single_dmp_test.exe"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines=True)

# 向C++程序发送输入数据
input_data = "hello"
cpp_process.stdin.write(input_data + "\n")    # 写入输入数据
cpp_process.stdin.flush()    # 刷新输入流

# 从C++程序接收输出数据
output_data = cpp_process.stdout.readline().strip()

# 打印输出结果
print("Output from C++ program:", output_data)

# 关闭C++程序的输入和输出流，并等待其结束
cpp_process.stdin.close()
cpp_process.stdout.close()
cpp_process.wait()