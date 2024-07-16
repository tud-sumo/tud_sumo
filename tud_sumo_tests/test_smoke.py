def test_smoke():
    """
    Test that the library is importable and has a version attribute.

    Due to the re-exports in the package init file, this will also
    effectively import everything else.
    """
    import tud_sumo
    assert tud_sumo.__version__
