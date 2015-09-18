nvcc -O3 -gencode arch=compute_30,code=sm_30 -odir "src" -M -o "src/main.d" "src/main.cpp"
nvcc -O3 --compile  -x c++ -o  "src/main.o" "src/main.cpp"

nvcc -O3 -gencode arch=compute_30,code=sm_30 -odir "src/Interface" -M -o "src/Interface/KDevice.d" "src/Interface/KDevice.cpp"
nvcc -O3 --compile  -x c++ -o  "src/Interface/KDevice.o" "src/Interface/KDevice.cpp"
nvcc -O3 -gencode arch=compute_30,code=sm_30 -odir "src/Interface" -M -o "src/Interface/KHost.d" "src/Interface/KHost.cpp"

nvcc -O3 --compile  -x c++ -o  "src/Interface/KHost.o" "src/Interface/KHost.cpp"
nvcc -O3 -gencode arch=compute_30,code=sm_30 -odir "src/Input" -M -o "src/Input/KSimulationData.d" "src/Input/KSimulationData.cpp"

nvcc -O3 --compile  -x c++ -o  "src/Input/KSimulationData.o" "src/Input/KSimulationData.cpp"
nvcc -O3 -gencode arch=compute_30,code=sm_30 -odir "src/Device" -M -o "src/Device/DeviceInterface.d" "src/Device/DeviceInterface.cu"

nvcc --compile -O3 -gencode arch=compute_30,code=compute_30 -gencode arch=compute_30,code=sm_30  -x cu -o  "src/Device/DeviceInterface.o" "src/Device/DeviceInterface.cu" 

nvcc --cudart static -link -o  "Release/V_02_2015"  src/main.o  src/Interface/KDevice.o src/Interface/KHost.o  src/Input/KSimulationData.o  src/Device/DeviceInterface.o   -lGL -lGLU -lglut
