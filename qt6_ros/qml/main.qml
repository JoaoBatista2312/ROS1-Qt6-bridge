import QtQuick 2.12
import QtQuick.Controls 2.5

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "ROS Subscriber with QML (Python)"

    Column {
        spacing: 20
        anchors.centerIn: parent

        Text {
            id: rosDataDisplay
            text: rosBridge.rosData  // Bind the text to rosBridge's rosData property
            font.pointSize: 24
        }

        Button {
            text: "Quit"
            onClicked: Qt.quit()
        }
    }
}
