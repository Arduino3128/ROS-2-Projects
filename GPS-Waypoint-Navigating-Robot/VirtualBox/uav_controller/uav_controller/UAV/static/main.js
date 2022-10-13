markerList = []
async function init() {
    const response = await fetch('/gps_data');
    const data = await response.json();
    console.log("First Fetch: " + data)
    map = new google.maps.Map(mapDiv = document.getElementById("map_canvas"), {
        mapTypeId: "satellite",
        zoom: 50,
        center: new google.maps.LatLng(data.latitude, data.longitude)
    });
    map.setHeading(data.angle);
    var chart_overlay = document.getElementById("chart_canvas")
    var chart_data = {
        labels: [],
        datasets: [{
            label: 'Speed',
            data: [40, 60],
            backgroundColor: [
                '#28a161',
                'rgba(0, 0, 0, 0.2)'
            ],
            borderWidth: 0,
        }]
    };
    var options = {
        rotation: 90,
        circumference: 270,
        showTooltips: false,
        responsive: true,
        cutout: 80,
    };
    chart = new Chart(chart_overlay, {
        type: 'doughnut',
        data: chart_data,
        options: options,
    });
    chart.options.animation = false;
    mapper();
}

async function mapper() {
    const response = await fetch('/gps_data');
    const data = await response.json();
    markerList.push(new google.maps.LatLng(data.latitude, data.longitude));
    initialize(data.latitude, data.longitude, data.heading, data.bearing, data.angle);
    await update_stats(data.speed, data.deltatheta, data.distance, data.satnum, data.latitude, data.longitude,)
    await graph(data.speed);
    map.setHeading(data.angle);
    setTimeout(mapper, 750);
}
function initialize(latitude, longitude, heading, bearing, angle) {
    try {
        polyline.setMap(null)
        heading_polyline.setMap(null)
        bearing_polyline.setMap(null)
        marker.setMap(null)
    } catch (error) {

    }
    pointer = document.getElementById("pointer");
    pointer.style.transform = "rotate(" + angle.toString() + "deg)";
    const symbolOne = {
        path: "M29.395,0H17.636c-3.117,0-5.643,3.467-5.643,6.584v34.804c0,3.116,2.526,5.644,5.643,5.644h11.759 c3.116,0,5.644-2.527,5.644-5.644V6.584C35.037,3.467,32.511,0,29.395,0z M34.05,14.188v11.665l-2.729,0.351v-4.806L34.05,14.188z M32.618,10.773c-1.016,3.9-2.219,8.51-2.219,8.51H16.631l-2.222-8.51C14.41,10.773,23.293,7.755,32.618,10.773z M15.741,21.713	v4.492l-2.73-0.349V14.502L15.741,21.713z M13.011,37.938V27.579l2.73,0.343v8.196L13.011,37.938z M14.568,40.882l2.218-3.336 h13.771l2.219,3.336H14.568z M31.321,35.805v-7.872l2.729-0.355v10.048L31.321,35.805z",
        strokeColor: "#000000",
        fillColor: "#0000FF",
        fillOpacity: 1,
        rotation: angle,
        scale: 0.8,
        origin: new google.maps.Point(0, 0),
        anchor: new google.maps.Point(25, 25),
        labelOrigin: new google.maps.Point(0, 0),
    };
    marker = new google.maps.Marker({
        position: new google.maps.LatLng(latitude, longitude),
        icon: symbolOne,
        map: map,
        center: new google.maps.LatLng(latitude, longitude)
    });
    polyline = new google.maps.Polyline({
        clickable: false,
        geodesic: true,
        strokeColor: "#6495ED",
        strokeOpacity: 1.000000,
        strokeWeight: 3,
        map: map,
        center: new google.maps.LatLng(latitude, longitude),
        path: markerList
    });

    heading_polyline = new google.maps.Polyline({
        clickable: false,
        geodesic: true,
        strokeColor: "#00FF00",
        strokeOpacity: 1.000000,
        strokeWeight: 3,
        map: map,
        path: [
            new google.maps.LatLng(latitude, longitude),
            new google.maps.LatLng(heading[0], heading[1]),
        ]
    });

    bearing_polyline = new google.maps.Polyline({
        clickable: false,
        geodesic: true,
        strokeColor: "#FF0000",
        strokeOpacity: 1.000000,
        strokeWeight: 3,
        map: map,
        path: [
            new google.maps.LatLng(latitude, longitude),
            new google.maps.LatLng(bearing[0], bearing[1]),
        ]
    });

    new google.maps.Circle({
        strokeColor: '#FF0000',
        strokeOpacity: 1.0,
        strokeWeight: 1,
        fillColor: '#FF0000',
        fillOpacity: 0.3,
        map: map,
        center: new google.maps.LatLng(latitude, longitude),
        radius: 0.4
    });

}

async function graph(speed) {
    var max_speed = 3;
    var chart_data = {
        labels: [],
        datasets: [{
            label: 'Speed',
            data: [speed * 100 / max_speed, 100 - speed * 100 / max_speed],
            backgroundColor: [
                '#28a161',
                'rgba(0, 0, 0, 0.2)'
            ],
            hoverOffset: 2,
            borderWidth: 1
        }]
    };
    chart.data = chart_data;
    chart.update()
}

async function update_stats(speed, deltatheta, dist, satnum, latitude, longitude) {
    document.getElementById("overlay_data").innerHTML = `UAV METADATA<br>
    Speed: `+ speed.toString() + ` km/h<br>
    Delta Theta: `+ deltatheta.toString() + `° <br>
    Target Distance: `+ dist.toString() + ` m<br>
    Satellite Count: `+ satnum.toString() + `<br>`;
    document.getElementById("right_overlay_data").innerHTML = `UAV INFORMATION<br>
    Latitude: <br>
    `+ latitude.toString() + `<br>
    Longitude: <br>
    `+ longitude.toString() + `<br>`;
}