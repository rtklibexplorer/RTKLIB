#ifndef gm_templateH
#define gm_templateH

#include <QString>

QString getHtmlPage() {
    return QStringLiteral(R"~~~(

<!------------------------------------------------------------------------------
* rtkplot_gm.htm: rtkplot google map view
*
* history: 2013/01/20 1.0  new
*------------------------------------------------------------------------------>
<html>
<head>
<title>RTKLIB_GM</title>

<script src="http://maps.google.com/maps/api/js?key=_APIKEY_&v=3"
    type="text/javascript" charset="UTF-8"></script>
<script type="text/javascript">

var map = null;
var marks = [];
var markh = null;
var markz = 0;
var info = null;
var icon0="http://maps.google.co.jp/mapfiles/ms/icons/red-dot.png";
var icon1="http://maps.google.co.jp/mapfiles/ms/icons/yellow-dot.png";

function init() {
    var opt = {
        center: new google.maps.LatLng(0,0),
        zoom: 2, minZoom: 2,
        streetViewControl: false,
        mapTypeId: google.maps.MapTypeId.ROADMAP
    };
    map = new google.maps.Map(document.getElementById("map"),opt);
    document.getElementById('state').value='1';
}

function SetView(lat,lon,zoom) {
    if (map == null) return;
    map.setCenter(new google.maps.LatLng(lat,lon));
    map.setZoom(zoom);
}

function SetCent(lat,lon) {
    if (map == null) return;
    map.setCenter(new google.maps.LatLng(lat,lon));
}

function SetZoom(zoom) {
    if (map == null) return;
    map.setZoom(zoom);
}

function ClearMark(lat,lon,title) {
    for (var i in marks) {
        marks[i].setMap(null);
    }
    marks.length = 0;
    markh = null;
}

function AddMark(lat,lon,title,msg) {
    var pos = new google.maps.LatLng(lat,lon);
    var opt = {map: map, position: pos, title: title, icon: icon1};
    var mark = new google.maps.Marker(opt);
    google.maps.event.addListener(mark,'click',function(event) {
        if (info) {info.close();}
        info = new google.maps.InfoWindow({content: msg});
        info.open(mark.getMap(),mark);
    });
    marks.push(mark);
}

function HighlightMark(title) {
    if (markh) {
        markh.setIcon(icon1);
        markh.setZIndex(markz);
    }
    for (var i in marks) {
        if (marks[i].getTitle()==title) {
            markz = marks[i].getZIndex();
            marks[i].setIcon(icon0);
            marks[i].setZIndex(google.maps.Marker.MAX_ZINDEX+1);
            markh=marks[i];
        }
    }
}

function PosMark(lat,lon,title) {
    for (var i in marks) {
        if (marks[i].title==title) {
            marks[i].setPosition(new google.maps.LatLng(lat,lon));
            break;
        }
    }
}

function ShowMark(title) {
    for (var i in marks) {
        if (marks[i].title==title) {
            marks[i].setVisible(true);
            break;
        }
    }
}

function HideMark(title) {
    for (var i in marks) {
        if (marks[i].title==title) {
            marks[i].setVisible(false);
            break;
        }
    }
}

</script>
</head>

<body style="margin: 0;"; scroll="no"; onload="init()">
    <div id="map" style="height: 100%; width: 100%;"> </div>
    <input id="state" type="hidden" value="0">
</body>
</html>
)~~~");
}
#endif
