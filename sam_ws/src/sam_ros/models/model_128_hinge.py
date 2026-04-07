import torch.nn as nn

class HingeClassifier(nn.Module):
    def __init__(self, class_num=3):
        super(HingeClassifier, self).__init__()
    
        # img size is [128, 128]
        # Convolutional layers
        self.conv1 = nn.Conv2d(3, 64, kernel_size=4, stride=2, padding=1)  # Output: 16 x 64 x 64
        self.bn1 = nn.BatchNorm2d(64)
        self.conv2 = nn.Conv2d(64, 64, kernel_size=4, stride=2, padding=1)  # Output: 32 x 32 x 32
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1)  # Output: 64 x 16 x 16
        self.bn3 = nn.BatchNorm2d(128)
        self.conv4 = nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=1)  # Output: 128 x 8 x 8
        self.bn4 = nn.BatchNorm2d(128)
        self.conv5 = nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1)  # Output: 128 x 8 x 8
        self.bn5 = nn.BatchNorm2d(128)
        self.conv6 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)  # Output: 256 x 8 x 8
        self.bn6 = nn.BatchNorm2d(256)
        self.conv7 = nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1)  # Output: 256 x 8 x 8
        self.bn7 = nn.BatchNorm2d(256)

        # Fully connected layers
        self.fc1 = nn.Linear(256 * 8 * 8, 1024)
        self.fc2 = nn.Linear(1024, class_num) # (0_full, 1_part, 2_else)

    def forward(self, x):
        x = self.bn1(self.conv1(x))
        x = nn.ReLU()(x)
        x = self.bn2(self.conv2(x))
        x = nn.ReLU()(x)
        x = self.bn3(self.conv3(x))
        x = nn.ReLU()(x)
        x = self.bn4(self.conv4(x))
        x = nn.ReLU()(x)
        x = self.bn5(self.conv5(x))
        x = nn.ReLU()(x)
        x = self.bn6(self.conv6(x))
        x = nn.ReLU()(x)
        x = self.bn7(self.conv7(x))
        x = nn.ReLU()(x)
        
        # Flatten the tensor for the fully connected layers
        x = x.view(x.size(0), -1)
        x = nn.ReLU()(self.fc1(x))
        x = self.fc2(x)
        return x