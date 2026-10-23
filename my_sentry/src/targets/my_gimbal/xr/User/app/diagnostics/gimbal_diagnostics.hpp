#pragma once

namespace LibXR
{
class RamFS;
}

namespace UserApp::Diagnostics
{

/** Register read-only gimbal troubleshooting commands in the terminal RamFS. */
void RegisterGimbalDiagnosticCommands(LibXR::RamFS& ramfs);

}  // namespace UserApp::Diagnostics
