from flask import (
    Flask,
    jsonify,
    render_template,
)
import json
from math import *
from waitress import serve
from flask_wtf.csrf import CSRFProtect
from flask_wtf.csrf import CSRFError
import os
from datetime import datetime, timedelta
from werkzeug.middleware.proxy_fix import ProxyFix

response = {
    "latitude": 0.0,
    "longitude": 0.0,
    "heading": 0.0,
    "bearing": 0.0,
    "angle": 0.0,
    "deltatheta": 0.0,
    "speed": 0.0,
    "satnum": 0,
    "distance": 0.0,
}
data = {
    "timestamp": 0,
    "latitude": 0.0,
    "longitude": 0.0,
    "heading": 0.0,
    "bearing": 0.0,
    "speed": 0.0,
    "satnum": 0,
    "distance": 0.0,
}

app = Flask(__name__, static_url_path="", static_folder="static")
app.secret_key = "eefef"  # os.environ["FLASK_SECRET_KEY"]
app.permanent_session_lifetime = timedelta(weeks=1)
app.config.update(
    SESSION_COOKIE_SECURE=True,
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE="Lax",
)
app.wsgi_app = ProxyFix(app.wsgi_app, x_for=1, x_host=1)

# -------------- CSRF PROTECTION --------------
csrf = CSRFProtect(app)
# ----------------------------------------------
@app.errorhandler(404)
def page_not_found(e):
    return render_template("404.html"), 404


@app.errorhandler(CSRFError)
def handle_csrf_error(e):
    return render_template("csrf_error.html"), 400


@app.route("/gps_data")
def gps_data():
    global response, data

    with open("data/data.json") as file:
        try:
            data = json.load(file)
        except:
            pass
    latitude = data["latitude"]
    longitude = data["longitude"]
    heading = data["heading"]
    bearing = data["bearing"]
    response["angle"] = data["heading"]
    response["speed"] = data["speed"]
    response["latitude"] = latitude
    response["longitude"] = longitude
    response["satnum"] = data["satnum"]
    response["distance"] = round(data["distance"], 3)
    heading_coords = [latitude, longitude]
    angle = radians(heading)
    heading_coords[0] += 0.0001 * cos(angle)
    heading_coords[1] += 0.0001 * sin(angle)
    response["heading"] = heading_coords
    bearing_coords = [latitude, longitude]
    angle = radians(bearing)
    headingError = bearing - heading
    if headingError < -180:
        headingError += 360
    if headingError > 180:
        headingError -= 360
    response["deltatheta"] = round(headingError, 3)
    bearing_coords[0] += 0.0001 * cos(angle)
    bearing_coords[1] += 0.0001 * sin(angle)
    response["bearing"] = bearing_coords
    return jsonify(response)


@app.route("/")
def home():
    return render_template("index.html")


def main():
    os.chdir("/home/atticus/ros2_ws/src/uav_controller/uav_controller/UAV")
    print("⎆ Web Server started.")
    # serve(app, host="0.0.0.0", port=8000, debug=True)
    app.run(host="0.0.0.0", port=8000, debug=True)
