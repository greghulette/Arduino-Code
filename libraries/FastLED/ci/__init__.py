"""FastLED CI tools and utilities.

Importing the ``ci`` package should make console output safe on Windows before
any entrypoint prints Unicode status text. The helper remains lightweight and is
best-effort only, so package import should never fail due to console setup.
"""

_ci_initialized = False


def _ensure_init() -> None:
    """Lazily configure UTF-8 console on first call. Thread-safe via GIL."""
    global _ci_initialized  # noqa: PLW0603
    if _ci_initialized:
        return
    _ci_initialized = True
    try:
        from ci.util.console_utf8 import configure_utf8_console

        configure_utf8_console()
    except KeyboardInterrupt:
        import _thread  # noqa: PLC0415

        _thread.interrupt_main()
        raise
    except Exception:
        # Never fail import due to console configuration
        pass


_ensure_init()
