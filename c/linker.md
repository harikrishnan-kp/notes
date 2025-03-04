In C programming, libraries allow you to reuse code across multiple programs. There are two main types of libraries:

- Static Libraries (.a in Linux, .lib in Windows)
    - The library code is copied into the executable at compile time
    - larger executables but does not require external dependencies at runtime.
    - Updatability:	Requires recompilation

- Dynamic (Shared) Libraries (.so in Linux, .dll in Windows)
    - The library code is loaded at runtime.
    - Produces smaller executables and allows updating the library without recompiling the application.
    - multiple programs can share the same .so file

## dynamic linking and static linking
- dynamic linking leave place holders in application,that will resolve during runtime
- we can use gcc flag `-static` for static linking
## whats is laoder
youtube resource: https://www.youtube.com/watch?v=YoyKDZlXCUM&list=PLIz6U0slZNq2TS1zSUjZHgxBjAJL4nb92