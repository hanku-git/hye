import torch
import torch.nn as nn
import torch.nn.functional as F

class MaskClassifier(nn.Module):
    def __init__(self, args):
        super(MaskClassifier, self).__init__()

        self.class_num = args
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(4, 64, kernel_size=4, stride=3)
        self.bn1 = nn.BatchNorm2d(64)
        self.conv2 = nn.Conv2d(64, 128, kernel_size=4, stride=2)
        self.bn2 = nn.BatchNorm2d(128)
        self.conv3 = nn.Conv2d(128, 128, kernel_size=4, stride=2)
        self.bn3 = nn.BatchNorm2d(128)
        self.conv4 = nn.Conv2d(128, 128, kernel_size=3, stride=2)
        self.bn4 = nn.BatchNorm2d(128)
        self.conv5 = nn.Conv2d(128, 128, kernel_size=3, stride=2)
        self.bn5 = nn.BatchNorm2d(128)
        self.conv6 = nn.Conv2d(128, 128, kernel_size=3, stride=1)
        self.bn6 = nn.BatchNorm2d(128)
        self.conv7 = nn.Conv2d(128, 128, kernel_size=3, stride=1)
        self.bn7 = nn.BatchNorm2d(128)

        # Fully connected layers
        self.fc1 = nn.Linear(128 * 5 * 5, 1024)  # feature dim : 5.5208...
        self.fc2 = nn.Linear(1024, self.class_num)  # 4개의 클래스 (0, 1, 2, 3)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))
        x = F.relu(self.bn5(self.conv5(x)))
        x = F.relu(self.bn6(self.conv6(x)))
        x = F.relu(self.bn7(self.conv7(x)))
        x = x.view(-1, 128 * 5 * 5)  # Flatten the tensor
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x