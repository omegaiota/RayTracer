#include "error_dialog.h"

#include "PROJ6850/viewer.h"

namespace PROJ6850 {

// Simple passthrough function to hide cass member function call
void showError(std::string errorString, bool fatal) {
  Viewer::showError(errorString, fatal);
}

}  // namespace PROJ6850
