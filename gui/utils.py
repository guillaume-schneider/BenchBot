from PyQt5.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat
from PyQt5.QtCore import QRegExp

STYLE_SHEET = """
QMainWindow, QWidget#mainScreen {
    background-color: #0f172a;
    color: #f8fafc;
    font-family: 'Inter', 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
}

/* ScrollBar */
QScrollBar:vertical {
    border: none;
    background: #0f172a;
    width: 10px;
    margin: 0px 0px 0px 0px;
}
QScrollBar::handle:vertical {
    background: #334155;
    min-height: 20px;
    border-radius: 5px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* Sidebar/Nav */
QWidget#sidebar {
    background-color: #1e293b;
    border-right: 1px solid #334155;
}

QPushButton#navButton {
    text-align: left;
    padding: 15px 20px;
    border: none;
    color: #94a3b8;
    background-color: transparent;
    font-size: 14px; 
    font-weight: 600;
    border-radius: 8px;
    margin: 4px 10px;
}

QPushButton#navButton:hover {
    background-color: #334155;
    color: #cbd5e1;
}

QPushButton#navButton:checked {
    background-color: #334155;
    color: #f8fafc;
    border-left: 3px solid #6366f1;
}

/* Header */
QLabel#headerLabel {
    font-size: 28px;
    font-weight: 800;
    color: #f8fafc;
}

/* Card */
QFrame[class="card"] {
    background-color: rgba(30, 41, 59, 0.7);
    border: 1px solid #334155;
    border-radius: 12px;
}

/* Form Elements */
QLineEdit, QTextEdit, QComboBox {
    background-color: #0f172a;
    border: 1px solid #334155;
    color: #f1f5f9;
    padding: 10px;
    border-radius: 8px;
    selection-background-color: #6366f1;
}

QPushButton#actionButton {
    background-color: #6366f1;
    color: white;
    border: none;
    padding: 10px 20px;
    border-radius: 8px;
    font-weight: 600;
}
QPushButton#actionButton:hover {
    background-color: #4f46e5;
}

QPushButton#dangerButton {
    background-color: rgba(239, 68, 68, 0.2);
    color: #ef4444;
    border: 1px solid #ef4444;
    border-radius: 8px;
    font-weight: 600;
}
QPushButton#dangerButton:hover {
    background-color: rgba(239, 68, 68, 0.3);
}
"""

class YamlHighlighter(QSyntaxHighlighter):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.rules = []
        key_format = QTextCharFormat()
        key_format.setForeground(QColor("#818cf8"))
        key_format.setFontWeight(QFont.Bold)
        self.rules.append((QRegExp(r"^\s*[\w.-]+(?=\s*:)"), key_format))
        self.rules.append((QRegExp(r"(?<=- )\s*[\w.-]+(?=\s*:)"), key_format))
        value_format = QTextCharFormat()
        value_format.setForeground(QColor("#34d399"))
        self.rules.append((QRegExp(r":\s+\".*\""), value_format))
        self.rules.append((QRegExp(r":\s+'.*'"), value_format))
        num_format = QTextCharFormat()
        num_format.setForeground(QColor("#fbbf24"))
        self.rules.append((QRegExp(r":\s+(true|false|null|\d+(\.\d+)?)"), num_format))
        comment_format = QTextCharFormat()
        comment_format.setForeground(QColor("#64748b"))
        self.rules.append((QRegExp(r"#.*"), comment_format))

    def highlightBlock(self, text):
        for expression, format in self.rules:
            index = expression.indexIn(text)
            while index >= 0:
                length = expression.matchedLength()
                self.setFormat(index, length, format)
                index = expression.indexIn(text, index + length)
