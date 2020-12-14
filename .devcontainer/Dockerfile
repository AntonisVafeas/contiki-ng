FROM mcr.microsoft.com/vscode/devcontainers/base:0-ubuntu-18.04

RUN dpkg --add-architecture i386

ENV DEBIAN_FRONTEND noninteractive

# Set user for what comes next
USER root

# Tools
RUN apt-get -qq update && \
  apt-get -qq -y --no-install-recommends install gnupg > /dev/null

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF && \
  echo "deb http://download.mono-project.com/repo/ubuntu bionic main" | tee /etc/apt/sources.list.d/mono-xamarin.list && \
  apt-get -qq update && \
  apt-get -qq -y --no-install-recommends install \
  ant \
  build-essential \
  default-jdk \
  doxygen \
  gdb \
  git \
  gtk-sharp2 \
  iputils-tracepath \
  iputils-ping \
  libcanberra-gtk-module:i386 \
  libgtk2.0-0 \
  mono-complete \
  mosquitto \
  mosquitto-clients \
  net-tools \
  npm \
  openjdk-8-jdk:i386 \
  python-pip \
  python-serial \
  rlwrap \
  sudo \
  screen \
  srecord \
  uml-utilities \
  unzip \
  valgrind \
  wget \
  smitools \
  snmp \
  snmp-mibs-downloader \
  > /dev/null \
  && apt-get -qq clean

# Install coap-cli
RUN npm -q install coap-cli@0.5.1 -g

# Install ARM toolchain
RUN wget -nv https://launchpad.net/gcc-arm-embedded/5.0/5-2015-q4-major/+download/gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2 && \
  tar xjf gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2 -C /tmp/ && \
  cp -f -r /tmp/gcc-arm-none-eabi-5_2-2015q4/* /usr/local/ && \
  rm -rf /tmp/gcc-arm-none-eabi-* gcc-arm-none-eabi-*-linux.tar.bz2

# Install msp430 toolchain
RUN wget -nv http://simonduq.github.io/resources/mspgcc-4.7.2-compiled.tar.bz2 && \
  tar xjf mspgcc*.tar.bz2 -C /tmp/ && \
  cp -f -r /tmp/msp430/* /usr/local/ && \
  rm -rf /tmp/msp430 mspgcc*.tar.bz2

# Install sphinx and sphinx_rtd_theme, required for building and testing the
# readthedocs API documentation
RUN pip -q install --upgrade pip
RUN pip -q install setuptools && pip -q install sphinx_rtd_theme sphinx

# Create user, enable X forwarding, add to group dialout
# -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix
RUN export uid=1000 gid=1000 && \
  mkdir -p /home/user && \
  echo "user:x:${uid}:${gid}:user,,,:/home/user:/bin/bash" >> /etc/passwd && \
  echo "user:x:${uid}:" >> /etc/group && \
  echo "user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
  chmod 0440 /etc/sudoers && \
  chown ${uid}:${gid} -R /home/user && \
  usermod -aG dialout user
# Make sure we're using Java 8 for Cooja
RUN update-java-alternatives --set /usr/lib/jvm/java-1.8.0-openjdk-i386

# Set user for what comes next
USER user

# Environment variables
ENV JAVA_HOME           /usr/lib/jvm/default-java
ENV HOME                /home/user
ENV CONTIKI_NG          ${HOME}/contiki-ng
ENV COOJA               ${CONTIKI_NG}/tools/cooja
ENV                     PATH="HOME:${PATH}"
WORKDIR                 ${CONTIKI_NG}

# Create Cooja shortcut
RUN echo "#!/bin/bash\nant -Dbasedir=${COOJA} -f ${COOJA}/build.xml run" > ${HOME}/cooja && \
  chmod +x ${HOME}/cooja

# Working directory
WORKDIR ${CONTIKI_NG}/examples/sphere/sphere-wearable3/screen-interfacing

# Start a bash
CMD bash --login
