"""Run the supported teleoperation and capture application."""
if __name__ == '__main__':
    from .bootstrap import _ensure_conda_cpp_runtime
    _ensure_conda_cpp_runtime()
    from .app.cli import main
    main()
