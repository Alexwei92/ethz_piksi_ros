import matplotlib.pyplot as plt

from collections import deque

BLACK  = (0,0,0)
WHITE  = (255,255,255)
RED    = (0,0,255)
BLUE   = (255,0,0)
GREEN  = (0,255,0)
YELLOW = (0,255,255)

class GPSMapClass():
    """
    plot a map with GPS coordinates
    """
    def __init__(self):
        self.fig, self.axis = plt.subplots()
        plt.ion()
        self.initialize_map()
        self.initialize_variables()

    def initialize_variables(self):
        self.has_initialized = False

        self.gps_history = deque(maxlen=50)

    def initialize_map(self):
        self.axis.set_xlabel('Latitude [deg]')
        self.axis.set_ylabel('Longitude [deg]')
        self.axis.set_aspect('equal')
        self.axis.set_xlim([-122.4100, -122.4200])
        self.axis.set_ylim([37.77, 37.78])
        self.axis.ticklabel_format(useOffset=False)
        self.fig.tight_layout()

    def update_graph(self, gps_data):
        # print(gps_data['longitude'], gps_data['latitude'])
        if not self.has_initialized:
            self.handler, = self.axis.plot(
                gps_data['longitude'],
                gps_data['latitude'],
                marker='o', markersize=5, fillstyle='full', color='b'
            )

            self.has_initialized = True

        else:
            self.handler.set_xdata(gps_data['longitude'])
            self.handler.set_ydata(gps_data['latitude'])

        self.gps_history.append(gps_data)