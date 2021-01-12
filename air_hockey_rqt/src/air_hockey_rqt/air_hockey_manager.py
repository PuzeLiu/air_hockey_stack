import os
import rospy
import rospkg

from argparse import ArgumentParser

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QTextCursor, QTextCharFormat, QBrush, QColor
from python_qt_binding.QtWidgets import QWidget, QDialogButtonBox


from air_hockey_referee.srv import *
from air_hockey_referee.msg import GameStatus


class AirHockeyManager(Plugin):
    def __init__(self, context):
        super(AirHockeyManager, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Air Hockey Manager')

        # Add argument(s) to the parser.
        parser = ArgumentParser()
        parser.add_argument("-v", "--verbose", action="store_true",
                            dest="verbose",
                            help="Put plugin in verbose mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if args.verbose:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('air_hockey_rqt'), 'resource', 'AirHockeyManager.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('AirHockeyManagerUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._set_services()

        self._game_status_topic = rospy.Subscriber("game_status", GameStatus, self._game_status_cb, queue_size=1)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def _set_services(self):
        start_srv_name = 'start_game'
        pause_srv_name = 'pause_game'
        stop_srv_name = 'stop_game'
        cancel_srv_name = 'reset_robot'

        self._start_srv = rospy.ServiceProxy(start_srv_name, StartGame, persistent=True)
        self._pause_srv = rospy.ServiceProxy(pause_srv_name, PauseGame, persistent=True)
        self._stop_srv = rospy.ServiceProxy(stop_srv_name, StopGame, persistent=True)
        self._reset_srv = rospy.ServiceProxy(cancel_srv_name, ResetRobot, persistent=True)

        self._connect_services_to_buttons()

    def _execute_service(self, service, request):
        try:
            res = service.call(request)

            c = self._widget.textBrowser.textCursor()

            if not res.success:
                format = QTextCharFormat()
                format.setForeground(QBrush(QColor("red")))
                c.setCharFormat(format)
            else:
                format = QTextCharFormat()
                format.setForeground(QBrush(QColor("black")))
                c.setCharFormat(format)

            c.insertText(res.msg + '\n')
            c.movePosition(QTextCursor.End)

            self._widget.textBrowser.setTextCursor(c)
        except Exception as e:
            c = self._widget.textBrowser.textCursor()
            format = QTextCharFormat()
            format.setForeground(QBrush(QColor("red")))
            c.setCharFormat(format)
            c.insertText('Service Error\n')
            c.insertText(str(e))
            c.movePosition(QTextCursor.End)

            self._widget.textBrowser.setTextCursor(c)

    def _start_cb(self):
        request = StartGameRequest()
        self._execute_service(self._start_srv, request)

    def _stop_cb(self):
        request = StopGameRequest()
        self._execute_service(self._stop_srv, request)

    def _pause_cb(self):
        request = PauseGameRequest()
        self._execute_service(self._pause_srv, request)

    def _reset_cb(self):
        request = ResetRobotRequest()
        self._execute_service(self._reset_srv, request)

    def _game_status_cb(self, status):
        self._widget.homeScore.display(status.score_home)
        self._widget.awayScore.display(status.score_away)

    def _connect_services_to_buttons(self):
        self._widget.startButton.clicked[bool].connect(self._start_cb)
        self._widget.pauseButton.clicked[bool].connect(self._pause_cb)
        self._widget.stopButton.clicked[bool].connect(self._stop_cb)
        self._widget.resetButton.clicked[bool].connect(self._reset_cb)
