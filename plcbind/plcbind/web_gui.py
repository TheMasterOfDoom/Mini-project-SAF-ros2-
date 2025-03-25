from flask import Flask, render_template # Flask
from markupsafe import escape

# ROS imports to setup the node
import rclpy



from plc_msgs.msg import CarierData

# Imports for threading operations
import sys
from threading import Thread
import atexit

plc_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


##### Setting up the ROS node:
def callback(msg):
    global new_image
    node.get_logger().info('I heard: "%s"' % msg)
    plc_stations[msg.stationid-1] = msg.carrierid



# Initializing the node
rclpy.init(args=None)
node = rclpy.create_node('web_gui')

# start the ROS node called Show_image_python in a new thread
Thread(target=lambda:node).start() # Starting the Thread with a target in the node
Thread(target=lambda:rclpy.spin(node)).start() # Starting the Thread with a target in the node
subscription = node.create_subscription(CarierData,'/plc_out', callback, 10) # Subscriber to the /image_name topic

# create flask app
app = Flask(__name__)

# spin ROS once and refresh the node
def get_image():
    return plc_stations

# main flask page gets the image and renders
@app.route('/')
def index():
    return index_html()

@app.route('/data')
def data():
    plc_stations = get_image()
    return f"{escape(plc_stations)}"


#defining function to run on shutdown
def close_running_threads():
    rclpy.shutdown()
    print("closed ROS")
    sys.exit(0)
    

## Main funcion, only initiate the Flask app
def main(args=None):
    atexit.register(close_running_threads) # call the function to close things properly when the server is down
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = -1
    app.run(host='0.0.0.0', port=5000 ,debug=False)


def index_html():
    return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Station Data</title>
    <style>
        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            background-color: #f4f4f4;
        }
        .circle-container {
            width: 300px;
            height: 300px;
            position: relative;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .station {
            position: absolute;
            width: 40px;
            height: 40px;
            line-height: 40px;
            text-align: center;
            background-color: lightblue;
            border-radius: 50%;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="circle-container" id="station-container">
        <!-- Stations will be inserted dynamically -->
    </div>
    <script>
        async function fetchData() {
            try {
                const response = await fetch('/data');
                const text = await response.text();
                const data = JSON.parse(text);
                
                const container = document.getElementById('station-container');
                container.innerHTML = '';
                
                const radius = 120;
                const centerX = 150;
                const centerY = 150;
                
                data.forEach((value, index) => {
                    const angle = (index / data.length) * 2 * Math.PI - Math.PI / 2;
                    const x = centerX + radius * Math.cos(angle) - 20;
                    const y = centerY + radius * Math.sin(angle) - 20;
                    
                    const station = document.createElement('div');
                    station.className = 'station';
                    station.style.left = `${x}px`;
                    station.style.top = `${y}px`;
                    station.textContent = value;
                    
                    container.appendChild(station);
                });
            } catch (error) {
                console.error('Error fetching data:', error);
            }
        }
        fetchData();
        setInterval(fetchData, 500); // Refresh every 5 seconds
    </script>
</body>
</html>
"""

if __name__ == '__main__':
    main()

