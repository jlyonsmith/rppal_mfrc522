FROM mcr.microsoft.com/devcontainers/rust:2-1-trixie
RUN apt-get update
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get -y install --no-install-recommends build-essential clang pkg-config libasound2-dev libclang-dev git-lfs cmake
ENV LIBCLANG_PATH=/usr/lib/llvm-19/lib
RUN git lfs install --system
