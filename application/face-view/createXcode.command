cd $(dirname "$0")
#rm -rf build && mkdir build
cd build
CC=/usr/local/opt/llvm/bin/clang CXX=/usr/local/opt/llvm/bin/clang++ cmake -G Xcode .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXECUTABLE=ON -DF2F_NO_CUDA=ON -DENABLE_CUDA=OFF -DWITH_IMGUI=ON -DFACE_TOOLKIT=ON
open prt_test.xcodeproj