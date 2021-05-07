FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install --yes gcc g++ cmake make wget unzip

RUN wget -O opencv-4.1.0.zip https://codeload.github.com/opencv/opencv/zip/4.1.0 && \
    unzip opencv-4.1.0.zip && \
    rm opencv-4.1.0.zip

RUN wget -O opencv_contrib-4.1.0.zip https://codeload.github.com/opencv/opencv_contrib/zip/4.1.0 && \
    unzip opencv_contrib-4.1.0.zip && \
    rm opencv_contrib-4.1.0.zip

RUN mkdir opencv-4.1.0/build && \
    cd opencv-4.1.0/build    && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.1.0/modules \
          -DBUILD_STATIC_LIBS=ON \
          -DBUILD_SHARED_LIBS=OFF \
          -DOPENCV_ENABLE_NONFREE=ON \
          .. && \
    make -j$(nproc --ignore 1) && \
    make install && \
    cd ../../ && \
    rm --force --recursive opencv-4.1.0/ opencv_contrib-4.1.0/

RUN mkdir --parents /workspace

WORKDIR /workspace

