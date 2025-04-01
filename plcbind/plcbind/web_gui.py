from flask import Flask # Flask
import json

# ROS imports to setup the node
import rclpy



from plc_msgs.msg import CarierData

# Imports for threading operations
import sys
from threading import Thread
import atexit

plc_stations = [{"Stationid":-1,"last_carrierid":-1, "read_time":"none"} for i in range(16)]


##### Setting up the ROS node:
def callback(msg):
    global new_image
    node.get_logger().info('I heard: "%s"' % msg)
    plc_stations[msg.stationid-1] = {"Stationid":msg.stationid,"last_carrierid":msg.carrierid, "read_time":msg.readtime}



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
    plc_stations = json.dumps(get_image())
    return plc_stations


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
    <title>Production Line</title>
        <style>
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: #f4f4f4;
        }
        .production-line {
            display: grid;
            grid-template-columns: repeat(11, 150px);
            grid-template-rows: repeat(7, 80px);
            gap: 5px;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
        }
        .station {
            display: flex;
            justify-content: center;
            align-items: center;
            border: 2px solid #333;
            border-radius: 5px;
            background: #fff;
            text-align: center;
            font-size: 14px;
            width: 150px;
            height: 80px;
            align-self: center;
            justify-self: center;
        }
        .conveyor-horizontal {
            background: #333;
            height: 20px;
            width: 150px;
            align-self: center;
            justify-self: center;
        }
        .conveyor-vertical {
            background: #333;
            width: 20px;
            height: 80px;
            justify-self: center;
            align-self: center;
        }
        .empty {
            grid-area: 2 / 2 / span 5 / span 9;
            border-radius: 10px;
            box-shadow: 0 0 -10px rgba(0, 0, 0, 0.1);
        }
    </style>
</head>
<body>
    <div class="production-line">
        <div class="station" id="station-1">1</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-2">2</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-3">3</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-4">4</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-5">5</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-6">6</div>
        
        <div class="conveyor-vertical"></div>
        <div class="empty"></div>
        <div class="conveyor-vertical"></div>
        
        <div class="station" id="station-16">16</div>
        <div class="station" id="station-7">7</div>
        
        <div class="conveyor-vertical"></div>
        <div class="conveyor-vertical"></div>
        
        <div class="station" id="station-15">15</div>
        <div class="station" id="station-8">8</div>
        
        <div class="conveyor-vertical"></div>
        <div class="conveyor-vertical"></div>
        
        <div class="station" id="station-14">14</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-13">13</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-12">12</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-11">11</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-10">10</div>
        <div class="conveyor-horizontal"></div>
        <div class="station" id="station-9">9</div>
    </div>
    <script>
        async function fetchData() {
            try {
                const response = await fetch('/data');
                const text = await response.text();
                const data = JSON.parse(text);
                
                data.forEach((value, index) => {
                    const station = document.getElementById(`station-${index + 1}`);
                    station.innerHTML = `
                    Station ID: ${value.Stationid}<br>
                    Carrier ID: ${value.last_carrierid}<br>
                    Read Time: <br> 
                    ${value.read_time}`;
                    
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

