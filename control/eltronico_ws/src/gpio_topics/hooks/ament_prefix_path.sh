# Add this isolated install prefix so the ament index can discover gpio_topics.
_colcon_prepend_unique_value AMENT_PREFIX_PATH "$COLCON_CURRENT_PREFIX"
