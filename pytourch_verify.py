import torch, torchvision
print("Torch version :", torch.__version__)
print("TorchVision    :", torchvision.__version__)
print("CUDA available :", torch.cuda.is_available())
if torch.cuda.is_available():
    print("GPU name       :", torch.cuda.get_device_name(0))
    x = torch.randn(8, 3, 224, 224, device='cuda')
    print("Tensor on GPU  :", x.device, "| shape:", x.shape)