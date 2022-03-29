#include <iostream>
#include "include/tools/tools.h"
#include "include/mapping/mapping.h"
int main() {
    tools tools;
    //load pcd file names
    tools.setFileName();
    tools.GetIntFileNames(tools.pcd_path,"pcd");
    tools.setStartEnd();
    mapping mapping(tools.start_id,tools.end_id,tools.file_names_);

    return 0;
}
