import time
import torch
import torch.nn as nn
import torch.nn.functional as F
from policy.models.resnet import resnet18
from config.config import cfg


class HCircularConv2d(nn.Module):
    """Conv2d with circular padding on width (azimuth) and zero padding on height (pitch).

    Wraps an existing nn.Conv2d (whose own padding is forced to 0) and performs the
    asymmetric padding manually before each forward pass. This preserves the inner
    conv's weight / init, so we can swap it in for any Conv2d in a pretrained ResNet.
    """

    def __init__(self, conv: nn.Conv2d):
        super().__init__()
        # Force inner conv padding to 0 — we handle padding ourselves.
        kh, kw = conv.kernel_size
        self.pad_h = (kh - 1) // 2
        self.pad_w = (kw - 1) // 2
        conv.padding = (0, 0)
        conv.padding_mode = 'zeros'  # unused, but keep tensor sane
        self.conv = conv

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # F.pad order: (W_left, W_right, H_top, H_bottom)
        if self.pad_w > 0:
            x = F.pad(x, (self.pad_w, self.pad_w, 0, 0), mode='circular')
        if self.pad_h > 0:
            x = F.pad(x, (0, 0, self.pad_h, self.pad_h), mode='constant', value=0.0)
        return self.conv(x)


def _wrap_convs_horizontal_circular(module: nn.Module) -> None:
    """Recursively replace every nn.Conv2d child with HCircularConv2d (in-place).

    Preserves the original Conv2d weight and bias (just changes how padding is applied).
    Skips 1x1 convs since they have no spatial padding.
    """
    for name, child in list(module.named_children()):
        if isinstance(child, nn.Conv2d):
            kh, kw = child.kernel_size
            if kh == 1 and kw == 1:
                continue  # nothing to pad
            wrapped = HCircularConv2d(child)
            setattr(module, name, wrapped)
        else:
            _wrap_convs_horizontal_circular(child)


# input: [in_channels, image_height, image_width] — default 384×96 ERP, 2 channels (depth + mask)
class ResNet18(nn.Module):
    def __init__(self, output_dim: int, in_channels: int = None):
        super(ResNet18, self).__init__()
        if in_channels is None:
            in_channels = int(cfg["image_channels"])
        self.cnn = resnet18(pretrained=False)
        self.cnn.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.cnn.output_layer = nn.Conv2d(512, output_dim, kernel_size=1, stride=1, padding=0, bias=False)
        # ERP panorama: wrap all 3x3+ convs to use horizontal circular padding.
        _wrap_convs_horizontal_circular(self.cnn)

    def forward(self, depth: torch.Tensor) -> torch.Tensor:
        return self.cnn(depth)


# Faster and smaller (input: [C, H/3, W/3])
class ResNet14(nn.Module):
    def __init__(self, output_dim: int, in_channels: int = None):
        super(ResNet14, self).__init__()
        if in_channels is None:
            in_channels = int(cfg["image_channels"])
        self.cnn = resnet18(pretrained=False)
        self.cnn.conv1 = nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.cnn.layer4 = nn.Sequential()
        self.cnn.output_layer = nn.Conv2d(256, output_dim, kernel_size=1, stride=1, padding=0, bias=False)
        _wrap_convs_horizontal_circular(self.cnn)

    def forward(self, depth: torch.Tensor) -> torch.Tensor:
        return self.cnn(depth)


def YopoBackbone(output_dim):
    return ResNet18(output_dim)


if __name__ == '__main__':
    H = int(cfg["image_height"])
    W = int(cfg["image_width"])
    C = int(cfg["image_channels"])
    net = YopoBackbone(64)
    input_ = torch.zeros((1, C, H, W))
    start = time.time()
    output = net(input_)
    print("output shape:", output.shape, "took", time.time() - start, "s")
