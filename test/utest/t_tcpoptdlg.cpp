// Exercise the actual Qt fields so bracketed endpoints survive save/reopen.
#include <QApplication>
#include <QComboBox>
#include <QSpinBox>
#include <cstdio>
#include "tcpoptdlg.h"

int main(int argc, char **argv)
{
    QApplication application(argc,argv);
    const QString paths[]={
        "127.0.0.1:2101", "caster.example:2101", ":2101",
        "[::1]:2101", "user:pa:ss@[2001:db8::1]:2101/MOUNT:STR;TEST",
        "user:pw@[fe80::1%en0]:2101/MOUNT", "[fe80::1%2]:2101",
        "[::ffff:192.0.2.1]:2101"
    };
    for (const auto &path:paths) {
        TcpOptDialog dialog(nullptr,TcpOptDialog::OPT_NTRIP_CLIENT);
        dialog.setPath(path);
        if (dialog.getPath()!=path) {
            std::fprintf(stderr,"round trip failed: %s -> %s\n",
                         qPrintable(path),qPrintable(dialog.getPath()));
            return 1;
        }
    }
    TcpOptDialog dialog(nullptr);
    auto *host=dialog.findChild<QComboBox *>("cBAddress");
    auto *port=dialog.findChild<QSpinBox *>("sBPort");
    if (!host||!port) return 1;
    for (int option:{TcpOptDialog::OPT_TCP_SERVER,TcpOptDialog::OPT_UDP_SERVER,
                    TcpOptDialog::OPT_NTRIP_CASTER_CLIENT,
                    TcpOptDialog::OPT_NTRIP_CASTER_SERVER,TcpOptDialog::OPT_TCP_CLIENT,
                    TcpOptDialog::OPT_UDP_CLIENT,TcpOptDialog::OPT_NTRIP_CLIENT,
                    TcpOptDialog::OPT_NTRIP_SERVER}) {
        dialog.setOptions(option);
        dialog.setPath("[::1]:2101");
        if (!host->isEnabled()||host->currentText()!="::1"||port->value()!=2101)
            return 1;
    }
    dialog.setPath("caster.example");
    if (host->currentText()!="caster.example") return 1;
    for (const QString &address:{QString("::1"),QString("[::1]"),
                               QString("fe80::1%en0"),QString("[fe80::1%2]")}) {
        dialog.setPath(address);
        const QString unbracketed=address.startsWith('[') ? address.mid(1,address.size()-2) : address;
        if (host->currentText()!=unbracketed||port->value()!=0) return 1;
    }
    host->setCurrentText("fe80::1%en0");
    port->setValue(2101);
    if (dialog.getPath()!="[fe80::1%en0]:2101") return 1;
    host->setCurrentText("[2001:db8::1]");
    if (dialog.getPath()!="[2001:db8::1]:2101") return 1;

    for (int option:{TcpOptDialog::OPT_TCP_SERVER,TcpOptDialog::OPT_UDP_SERVER,
                    TcpOptDialog::OPT_NTRIP_CASTER_CLIENT,TcpOptDialog::OPT_NTRIP_CASTER_SERVER}) {
        TcpOptDialog listener(nullptr,option);
        auto *address=listener.findChild<QComboBox *>("cBAddress");
        if (!address) return 1;
        listener.setPath("[::1]:2101");
        address->setCurrentText("");
        if (!QMetaObject::invokeMethod(&listener,"accept",Qt::DirectConnection)) return 1;
        if (!address->isEnabled()||!address->currentText().isEmpty()||listener.getPath()!=":2101")
            return 1;
    }
    return 0;
}
