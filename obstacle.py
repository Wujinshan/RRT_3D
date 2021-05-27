class Obstacle():
          """
          There are 6 members in a cubic obstacle, the x, y and z represent the coordinate
          of the obstacle center, L:length; W:width;H:height
          """
          def __init__(self, x:float, y:float, z:float, L:float, W:float, H:float):
                    self.x = x
                    self.y = y
                    self.z = z
                    self.L = L
                    self.W = W
                    self.H = H
