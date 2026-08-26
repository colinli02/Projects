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

### Simple CNN & BP Implementation (2025)

[View Notebook](Notebooks/Simple_CNN_Notebook.ipynb)

- Implemented a convolutional neural network from scratch in **PyTorch**, including 2D convolution, ReLU, batch normalization, and their corresponding gradient operators
- Implemented back-propagation manually and verified gradients using **finite-difference gradient checking** against PyTorch autograd
- Built a two-layer CNN for **MNIST classification** and implemented gradient computation for convolutional filters and the linear output layer

### CUDA Transformer Sequence Classifier (2025)

[View Notebook](Notebooks/CUDA_TFM_Classifier_Notebook.ipynb)

- Implemented a Transformer-based classifier in PyTorch, including tokenization, absolute/relative positional encoding, multi-head self-attention, and Transformer layers
- Built the model architecture and training pipeline from scratch with Adam optimization, learning-rate warmup/cooldown scheduling, and configurable pre/post-normalization
- Trained the Transformer to classify synthetic sequence data, achieving 99.9% test accuracy
- Implemented and validated core components through unit testing, including attention, positional encoding, Transformer layers, optimization, and loss computation

## MNIST VAE (2025)

[View Notebook](Notebooks/MNIST_VAE_Notebook.ipynb)

- Implemented a Variational Autoencoder (VAE) in PyTorch with a convolutional encoder and decoder for MNIST image generation
- Implemented the reparameterization trick, Gaussian log-likelihood, and KL divergence to construct the ELBO training objective
- Trained the VAE using GPU acceleration and evaluated reconstruction quality through ELBO training curves
- Generated new MNIST samples by sampling from the latent prior and decoding the resulting latent representations