import torch.nn as nn

class HingeAngleModel(nn.Module):
    def __init__(self):
        super(HingeAngleModel, self).__init__()
    
        # Convolutional layers
        self.conv1 = nn.Conv2d(3, 64, kernel_size=4, stride=2, padding=1)  # Output: 64 x 64 x 64
        self.conv2 = nn.Conv2d(64, 64, kernel_size=4, stride=2, padding=1)  # Output: 64 x 32 x 32
        self.conv3 = nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1)  # Output: 128 x 16 x 16
        self.conv4 = nn.Conv2d(128, 128, kernel_size=4, stride=2, padding=1)  # Output: 128 x 8 x 8
        self.conv5 = nn.Conv2d(128, 128, kernel_size=3, stride=1, padding=1)  # Output: 128 x 8 x 8
        self.conv6 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)  # Output: 256 x 8 x 8
        self.conv7 = nn.Conv2d(256, 256, kernel_size=3, stride=1, padding=1)  # Output: 256 x 8 x 8

        # Fully connected layers
        self.fc1 = nn.Linear(256 * 8 * 8, 1024)
        self.fc2 = nn.Linear(1024, 360)  # 36 classes for classification

    def forward(self, x):
        x = self.conv1(x)
        x = nn.ReLU()(x)
        x = self.conv2(x)
        x = nn.ReLU()(x)
        x = self.conv3(x)
        x = nn.ReLU()(x)
        x = self.conv4(x)
        x = nn.ReLU()(x)
        x = self.conv5(x)
        x = nn.ReLU()(x)
        x = self.conv6(x)
        x = nn.ReLU()(x)
        x = self.conv7(x)
        x = nn.ReLU()(x)
        
        # Flatten the tensor for the fully connected layers
        x = x.view(x.size(0), -1)
        x = nn.ReLU()(self.fc1(x))
        x = self.fc2(x)  # No activation function here since CrossEntropyLoss will be applied
        return x
