import torch



def check_cuda_availability():
    if torch.cuda.is_available():
        print("CUDA is available. 🚀")
        print(f"Number of CUDA devices: {torch.cuda.device_count()}")
        for i in range(torch.cuda.device_count()):
            print(f"CUDA Device {i}: {torch.cuda.get_device_name(i)}")
            print(f"  Memory Allocated: {torch.cuda.memory_allocated(i)} bytes")
            print(f"  Memory Cached: {torch.cuda.memory_reserved(i)} bytes")
    else:
        print("CUDA is not available. 😞")
        print("Troubleshooting Tips:")
        print("1. Ensure you have a CUDA-capable GPU.")
        print("2. Make sure you have the latest driver installed for your GPU.")
        print("3. Ensure you have CUDA installed. You can download it from the NVIDIA website.")
        print("4. If you're using PyTorch, ensure you've installed the CUDA version of PyTorch.")
        print("5. Restart your system to ensure all installations are properly set up.")
        print("6. Check the environment variables related to CUDA (like CUDA_HOME) and ensure they're correctly set.")

if __name__ == "__main__":

    print(f"PyTorch Version: {torch.__version__}")

    # Check CUDA availability
    if torch.cuda.is_available():
        print("CUDA is available in PyTorch. 🚀")
        print(f"CUDA Version: {torch.version.cuda}")
        print(f"Number of CUDA Devices: {torch.cuda.device_count()}")
        for i in range(torch.cuda.device_count()):
            print(f"  Device {i}: {torch.cuda.get_device_name(i)}")
    else:
        print("CUDA is not available in PyTorch. 😞")
    check_cuda_availability()