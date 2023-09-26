FROM ros:noetic-perception

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y wget software-properties-common python3-catkin-tools libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libfmt-dev psmisc ros-noetic-tf-conversions libboost-all-dev python3-pybind11 libzip-dev python3-pip libspdlog-dev ros-noetic-rviz && \
    rm -rf /var/lib/apt/lists/

RUN wget ceres-solver.org/ceres-solver-2.1.0.tar.gz
RUN tar zxf ceres-solver-2.1.0.tar.gz
RUN mkdir ceres-bin
WORKDIR /ceres-bin
RUN cmake ../ceres-solver-2.1.0
RUN make -j8
RUN make install

RUN pip3 install numpy --upgrade
RUN pip3 install scikit-learn seaborn tqdm tabulate

WORKDIR / 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

USER root
WORKDIR /
RUN source /ros_entrypoint.sh
RUN echo "source /ros_entrypoint.sh" >> .bashrc
