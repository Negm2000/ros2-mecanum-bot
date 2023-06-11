from PyQt5 import QtCore, QtWidgets, QtGui
import time

from ProjectControls import getLocationInfo, LocationsNames, SpeakingThreshold
from HelperComponents import AnswerQuestion, initSpeech, saySpeech
from SpeechRecorder import SpeechRecorder

TEXT_STYLE = """QLabel {
    background: #262626;
    border-radius: 40px;
    font-size: 20pt;
    color: #F2DE79;
    padding: 20px;
    }
"""
HEADER_TEXT_STYLE = """QLabel {
    background: #262626;
    border-radius: 40px;
    font-size: 30pt;
    color: #F2DE79;
    padding: 20px;
    }
"""
ACTIVE_HEADER_TEXT_STYLE = """QLabel {
    background: #33BBA4;
    border-radius: 40px;
    font-size: 30pt;
    color: #E7E7E7;
    padding: 20px;
    }
"""

PUSH_BUTTON_STYLE = """QPushButton {
    font-size: 30pt;
    border: 1px solid;
    border-color: #F2C166;
    color: #F2C166;
    border-radius: 8px;
    margin: 10px 30px;
    padding: 10px;}
    
    QPushButton:hover {
    background-color: #A6763C;
    color: #262626;
    }
    QPushButton:pressed {
    background-color: #0D0D0D;
    color: #262626;
    }
"""


class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)

        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("application main window")
        self.main_widget = QtWidgets.QWidget(self)
        self.layout_main = QtWidgets.QGridLayout(self.main_widget)
        self.layout_main.setContentsMargins(0, 0, 0, 0)
        self.main_widget.setStyleSheet("""background: #0D0D0D;""")

        self.layout_text = QtWidgets.QHBoxLayout()
        self.layout_text.setContentsMargins(30, 30, 30, 0)
        self.HeaderTextbox = QtWidgets.QLabel()
        self.HeaderTextbox.setWordWrap(True)
        self.BoldFont = QtGui.QFont()
        self.BoldFont.setBold(True)
        self.HeaderTextbox.setFont(self.BoldFont)
        self.HeaderTextbox.setStyleSheet(HEADER_TEXT_STYLE)
        self.HeaderTextbox.setAlignment(QtCore.Qt.AlignCenter)
        self.layout_text.addWidget(self.HeaderTextbox)

        self.layout_actions = QtWidgets.QGridLayout()
        self.layout_actions.setContentsMargins(30, 0, 30, 0)

        self.layout_main.addLayout(self.layout_text, 0, 0, 1, 1)
        self.layout_main.addLayout(self.layout_actions, 1, 0, 1, 1)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)
        self.resize(1080, 720)

        self.DoYouWantTourPage()

        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.CurrentLocation = ''

    def GetActionButton(self, caption):
        butn = QtWidgets.QPushButton()
        butn.setText(caption)
        butn.setMinimumWidth(250)
        butn.setMinimumHeight(80)
        butn.setStyleSheet(PUSH_BUTTON_STYLE)
        return butn

    def DoYouWantTourPage(self):
        self.HeaderTextbox.setText('Hi! Do you want me to give you a tour?')
        # self.layout_main.setRowStretch(0, 3)
        # self.layout_main.setRowStretch(1, 1)

        self.AcceptButton = self.GetActionButton('YES')
        self.AcceptButton.clicked.connect(self.WhereToGoPage)
        self.layout_actions.addWidget(
            self.AcceptButton, 1, 1, QtCore.Qt.AlignCenter)

    def removeDoYouWantTourPage(self):
        self.layout_actions.removeWidget(self.AcceptButton)
        self.AcceptButton.deleteLater()

    def WhereToGoPage(self):
        self.removeDoYouWantTourPage()
        self.HeaderTextbox.setText('Where do you want to go?')
        self.HeaderTextbox.setMaximumHeight(200)
        self.SectionButtonsMapper = QtCore.QSignalMapper()

        MuseumSections = LocationsNames

        self.WhereToGoButtons = []
        i = 0
        row = 0
        self.SectionButtonsMapper.mapped[str].connect(self.ComePage)
        for section in MuseumSections:
            secButton = QtWidgets.QPushButton()
            secButton.setText(section)
            secButton.setContentsMargins(50, 20, 50, 20)
            secButton.setStyleSheet(PUSH_BUTTON_STYLE)
            # secButton.setMinimumWidth(100)
            self.WhereToGoButtons.append(secButton)
            self.SectionButtonsMapper.setMapping(secButton, section)
            secButton.clicked.connect(self.SectionButtonsMapper.map)
            self.layout_actions.addWidget(secButton, row, i % 3)
            if i % 3 == 2:
                row += 1
            i += 1

        QtWidgets.qApp.processEvents()
        saySpeech('where to go.mp3', False)

    def removeWhereToGoPage(self):
        for button in self.WhereToGoButtons:
            self.layout_actions.removeWidget(button)
            button.deleteLater()

    def ComePage(self, location):
        self.removeWhereToGoPage()
        self.HeaderTextbox.setText('Come with me please')
        self.HeaderTextbox.setMaximumHeight(20000)
        self.layout_text.setContentsMargins(30, 30, 30, 30)
        QtWidgets.qApp.processEvents()
        saySpeech('come please.mp3', False)
        ####################TODO#########################
        # connect wit ROS by send location to go to and
        # then calling self.SectionInfoPage(location)
        # the next line is just for demonstration purposes
        self.timer.singleShot(1000, lambda: self.SectionInfoPage(location))

    def ResetEverything(self):
        self.layout_text.setContentsMargins(30, 30, 30, 0)
        self.HeaderTextbox.setMaximumHeight(200)
        self.HeaderTextbox.setStyleSheet(HEADER_TEXT_STYLE)

    def SectionInfoPage(self, location):
        locationInfo = getLocationInfo(location)
        self.HeaderTextbox.setStyleSheet(TEXT_STYLE)
        self.HeaderTextbox.setText(locationInfo)

        def OnInfoSectionLoaded(text):
            path = initSpeech(text)
            saySpeech(path)
            self.timer.singleShot(
                2000, lambda: self.AnyQuestionsPage(location))
        self.timer.singleShot(
            100, lambda: OnInfoSectionLoaded(locationInfo))

    def AnyQuestionsPage(self, location):
        self.ResetEverything()
        self.HeaderTextbox.setText("Do you have any questions?")
        self.AcceptButton = self.GetActionButton('YES')
        self.AcceptButton.clicked.connect(
            lambda: self.QuestionAnswerPage(location))
        self.DeclineButton = self.GetActionButton('NO')
        self.DeclineButton.clicked.connect(self.declineAction)

        self.layout_buttons = QtWidgets.QHBoxLayout()
        self.layout_buttons.addWidget(self.AcceptButton)
        self.layout_buttons.addWidget(self.DeclineButton)
        self.layout_actions.addLayout(self.layout_buttons, 1, 0, 1, 1)
        QtWidgets.qApp.processEvents()
        saySpeech('do you have questions.mp3', False)

    def declineAction(self):
        self.removeAnyQuestionsPage()
        self.WhereToGoPage()

    def removeAnyQuestionsPage(self):
        self.layout_buttons.removeWidget(self.AcceptButton)
        self.layout_buttons.removeWidget(self.DeclineButton)
        self.AcceptButton.deleteLater()
        self.DeclineButton.deleteLater()

    def QuestionAnswerPage(self, location):
        self.layout_text.setContentsMargins(30, 30, 30, 30)
        self.HeaderTextbox.setMaximumHeight(20000)
        self.removeAnyQuestionsPage()
        self.HeaderTextbox.setText('please, say your question')

        def ListenToSpeech(location):
            recorder = SpeechRecorder(SpeakingThreshold)
            recorder.onRecording += self.ChangeColor
            answer = AnswerQuestion(location, recorder)
            answer = answer.replace('Teaty', 'Titi')

            self.HeaderTextbox.setText(answer)
            path = initSpeech(answer)
            QtWidgets.QApplication.processEvents()
            saySpeech(path)
            time.sleep(2)
            self.AnyQuestionsPage(location)
            del recorder

        self.timer.singleShot(1, lambda: ListenToSpeech(location))

    def ChangeColor(self, isSpeaking):
        if (isSpeaking):
            self.HeaderTextbox.setStyleSheet(ACTIVE_HEADER_TEXT_STYLE)
        else:
            self.HeaderTextbox.setStyleSheet(HEADER_TEXT_STYLE)
        QtWidgets.QApplication.processEvents()
        self.HeaderTextbox.repaint()
