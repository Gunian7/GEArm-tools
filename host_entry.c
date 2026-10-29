/* Native Windows test runner without dependency on missing MinGW CRT headers. */
__declspec(dllimport) void __cdecl exit(int);
int main(void);
void __main(void) {}
void mainCRTStartup(void) { exit(main()); }
