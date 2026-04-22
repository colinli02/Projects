# ML Projects

## Building Pytorch from Source (2025)

Built PyTorch from source on Windows 10 using Microsoft Visual Studio 2022 and CMake
configuring the build for CUDA 12.4 support and optimizing GPU utilization for deep learning tasks

## CUDA TCP Project (2025)

- Built a multithreaded C++ TCP client-server system on Windows using Winsock2, enabling concurrent handling of multiple clients via per-connection threads
- Integrated CUDA kernels to offload vector addition and matrix multiplication to the GPU, demonstrating basic heterogeneous computing
- Designed a command-based protocol allowing clients to trigger GPU computations and receive real-time results over network sockets
- Implemented thread-safe tracking of active client connections using atomic operations in a concurrent TCP server

[Repo Link](https://github.com/colinli02/CUDA_TCP)
??? "Click to view README.md"
    --8<-- "https://raw.githubusercontent.com/colinli02/CUDA_TCP/refs/heads/master/README.md"

### Conditional PixelCNN (2025)

- Converted an unconditional PixelCNN into a conditional generative model in **PyTorch** by integrating class embeddings and implementing **middle fusion** for label conditioning  
- Modified core training and inference pipelines (loss function, sampling, and evaluation) to support conditional image generation and classification  
- Achieved **81.3% test accuracy** and **26.8 FID score**, improving performance over a baseline embedding-only model (~75% accuracy, FID ~30)  
- Implemented likelihood-based classification using **discretized mixture logistic loss**, enabling joint generation and inference from the same model  
- Evaluated model performance using **Weights & Biases** and external benchmarks via **Hugging Face**, selecting optimal checkpoints to mitigate overfitting

## Simple CNN & BP Implementation

## CUDA based MNIST Transformer

## MNIST VAE
