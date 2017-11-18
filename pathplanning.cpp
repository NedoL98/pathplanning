#include <iostream>
#include "tinyxml.cpp"
#include "tinyxml.h"
#include "tinyxmlerror.cpp"
#include "tinyxmlparser.cpp"
#include "tinystr.cpp"
#include "tinystr.h"

using namespace std;

int main() {
    TiXmlDocument doc("/../../PathPlanning/example.xml");
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
    doc.SaveFile("/../../PathPlanning/example_new.xml"); //"map" section is deleted
    return 0;
}
