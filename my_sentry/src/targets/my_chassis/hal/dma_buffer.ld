SECTIONS
{
  .dma_buffer (NOLOAD) : ALIGN(32)
  {
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
  } >RAM_D3
}
