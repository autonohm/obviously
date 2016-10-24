FROM ubuntu

MAINTAINER Christian Pfitzner "christian.pfitzner@th-nuernberg.de"


RUN apt-get update

# install dependencies for building 
RUN apt-get install -y git
RUN apt-get install -y cmake 
RUN apt-get install -y libproj-dev

# install dependencies for obviously
RUN apt-get install -y libopenni-dev
RUN apt-get install -y build-essential
RUN apt-get install -y libgsl0-dev 
RUN apt-get install -y libudev-dev
RUN apt-get install -y libxml++2.6-dev
RUN apt-get install -y liblua5.1-0-dev 
RUN apt-get install -y libeigen3-dev 

RUN apt-get install -y libvtk6-dev
RUN apt-get install -y libvtk6.2 
RUN apt-get install -y freeglut3-dev

RUN apt-get install -y libann-dev
RUN apt-get install -y libflann-dev
RUN apt-get install -y libv4l-dev 


# add user 
RUN useradd -ms /bin/bash user
USER user
WORKDIR /home/user

RUN cd /home/user

# clone repositorie
RUN git clone https://github.com/autonohm/obviously.git 

RUN cd /home/user/obviously/build/release 
ENV OBVIOUSLY_ROOT=/home/user/obviously
WORKDIR /home/user/obviously/build/release 


# compile obviously release
RUN cmake .
RUN make -j2


  
