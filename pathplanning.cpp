#include <iostream>
#include "tinyxml.h"

using namespace std;

int main() {
    TiXmlDocument doc("example.xml");
    if (!doc.LoadFile()) {
        cout << "Load failed" << endl;
        return 0;
    }
    TiXmlNode *it = doc.RootElement()->FirstChild();
    while (it != NULL) {
        cout << it->Value() << endl;
        it = it->NextSibling();
    }
    doc.RootElement()->RemoveChild(doc.RootElement()->FirstChild());
    doc.SaveFile("../pathplanning/example_new.xml"); //"map" section is deleted
    return 0;
}
