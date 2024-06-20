from flask import Flask, render_template, request, jsonify
from datetime import datetime, timedelta
import sqlite3
import json
import threading
import time
import logging


# Set up logging
logging.basicConfig(level=logging.ERROR, format='%(asctime)s %(levelname)s:%(message)s')


app = Flask(__name__)

position_data = {}
position_data_lock = threading.Lock()
processed_data = []
def get_db_connection():
    conn = sqlite3.connect('environmental_data.db')
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    db = get_db_connection()
    db.execute('''
        CREATE TABLE IF NOT EXISTS environmental_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME,
            temp REAL,
            pressure REAL,
            humidity REAL,
            light REAL,
            oxidising REAL,
            reducing REAL,
            nh3 REAL,
            pm1 INTEGER,
            pm2_5 INTEGER,
            pm10 INTEGER
        )
    ''')
    db.commit()
    db.close()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/data")
def data():
    db = get_db_connection()
    time_limit = datetime.now() - timedelta(minutes=5)
    query = 'SELECT * FROM environmental_data WHERE timestamp >= ? ORDER BY timestamp DESC'
    rows = db.execute(query, (time_limit,)).fetchall()
    db.close()
    data_list = [dict(row) for row in rows]
    return jsonify({'data_list': data_list})  # Return data as JSON

@app.route("/view-live-data")
def view_data():
    db = get_db_connection()
    time_limit = datetime.now() - timedelta(minutes=1)
    # Query to fetch data from the last 5 minutes, sorted by timestamp in descending order
    query = 'SELECT * FROM environmental_data WHERE timestamp >= ? ORDER BY timestamp DESC'
    rows = db.execute(query, (time_limit,)).fetchall()
    db.close()
    data_list = [dict(row) for row in rows]
    return render_template('data.html', data_list=data_list)

@app.route("/render-timestamp-data", methods=['POST'])
def render_data():
    timestamp = request.form.get('timestamp')
    pos_x = request.form.get('x')
    pos_y = request.form.get('y')
    pos_z = request.form.get('z')
    
    pos = {
        'x': pos_x,
        'y': pos_y,
        'z': pos_z
    }
    print(pos)
    print(timestamp)
    with position_data_lock:
        position_data[timestamp] = pos

    return "Status: Received pos data"

def process_position_data():
    while True:
        try:
            with position_data_lock:
                if position_data:
                    print("Inside threading")
                    timestamp_str, pos = next(iter(position_data.items()))
                    print(timestamp_str)
                    print(pos)
                    try:
                        # Convert the timestamp string to a datetime object
                        timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                        print("Time stamp" + str(timestamp))
                        timestamp -= timedelta(hours=4)
                        print("Updated TimeStamp" + str(timestamp))
                        if datetime.now() >= timestamp + timedelta(seconds=5):
                            del position_data[timestamp_str]

                            db = get_db_connection()
                            time_limit = timestamp - timedelta(seconds=5)
                            query = 'SELECT * FROM environmental_data WHERE timestamp BETWEEN ? AND ?'
                            
                            # Ensure timestamp_datetime is defined correctly
                            timestamp_datetime = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
                            rows = db.execute(query, (time_limit, timestamp_datetime)).fetchall()
                            db.close()

                            if rows:
                                average_data = {key: sum(row[key] for row in rows) / len(rows) for key in rows[0].keys() if key != 'id' and key != 'timestamp'}
                                processed_entry = {
                                    'timestamp': timestamp_str,
                                    'position': pos,
                                    'average_data': average_data
                                }
                                processed_data.append(processed_entry)

                    except ValueError as ve:
                        logging.error(f"Error converting timestamp: {ve}")
                    except Exception as e:
                        logging.error(f"Error processing position data: {e}")
                        
            # print(processed_data)
            time.sleep(1)
            
        except Exception as e:
            logging.error(f"Unexpected error in process_position_data: {e}")
            time.sleep(1)  # Adding a slight delay before retrying


@app.route("/view-timestamp-data")
def view_timestamp_data():
    return render_template('timestamp.html', processed_data=processed_data)

@app.route("/health")
def health():
    return jsonify({"message": "Server is running"}), 200

@app.route("/send-data", methods=["POST"])
def fetch_data():
    data = request.json
    timestamp = datetime.now()
    db = get_db_connection()
    db.execute('''
        INSERT INTO environmental_data (timestamp, temp, pressure, humidity, light, oxidising, reducing, nh3, pm1, pm2_5, pm10)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    ''', (timestamp, data['temp'], data['pressure'], data['humidity'], data['light'], data['oxidising'],
          data['reducing'], data['nh3'], data['pm1'], data['pm2.5'], data['pm10']))
    db.commit()
    db.close()
    return jsonify({"message": "Environmental data stored successfully"}), 200

if __name__ == "__main__":
    init_db()
    threading.Thread(target=process_position_data, daemon=True).start()
    app.run(host='0.0.0.0', debug=True)
