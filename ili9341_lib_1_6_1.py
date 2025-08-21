"""
ILI9341 TFT SPI LCD module.
Written by Tom A. Aizenshtark

ILI9341 TFT SPI display activation library, made for the micropython implementation.
"""

from time import sleep
from framebuf import FrameBuffer, RGB565  # type: ignore
from micropython import const  # type: ignore
import gc


def color565(r, g, b):
    """Return RGB565 color value.

    Args:
        r (int): Red value.
        g (int): Green value.
        b (int): Blue value.
    """
    return (r & 0xf8) << 8 | (g & 0xfc) << 3 | b >> 3


def color888(r: int, g: int, b: int) -> int:
    """Return RGB888 color value.

    Args:
        r (int): Red value.
        g (int): Green value.
        b (int): Blue value.
    """
    return r << 16 | g << 8 | b


class Display:
    """Serial interface for ILI9341 display.
    
    Note:  All coordinates are zero based.
    """
    
    # Command constants from ILI9341 datasheet
    NOP = const(b"\x00")  # No-op
    SWRESET = const(b"\x01")  # Software reset
    RDDIDIF = const(b"\x04")  # Read display ID info
    RDDST = const(b"\x09")  # Read display status
    RDDPM = const(b"\x0A")  # Read display power mode
    RDDMADCTL = const(b"\x0B")  # Read display MADCTL
    RDDCOLMOD = const(b"\x0C")  # Read display pixel format
    RDDIM = const(b"\x0D")  # Read display image format
    RDDSDR = const(b"\x0F")  # Read display self-diagnostic result
    SLPIN = const(b"\x10")  # Enter sleep mode
    SLPOUT = const(b"\x11")  # Exit sleep mode
    PTLON = const(b"\x12")  # Partial mode OFF
    NORON = const(b"\x13")  # Normal display mode on
    DINVOFF = const(b"\x20")  # Display inversion OFF
    DINVON = const(b"\x21")  # Display inversion ON
    GAMSET = const(b"\x26")  # Gamma set
    DISPOFF = const(b"\x28")  # Display OFF
    DISPON = const(b"\x29")  # Display ON
    CASET = const(b"\x2A")  # Column address set
    PASET = const(b"\x2B")  # Page address set
    RAMWR = const(b"\x2C")  # Memory write
    RAMRD = const(b"\x2E")  # Memory read
    PLTAR = const(b"\x30")  # Partial area
    VSCRDEF = const(b"\x33")  # Vertical scrolling definition
    MADCTL = const(b"\x36")  # Memory access control
    VSCRSADD = const(b"\x37")  # Vertical scrolling start address
    IDMOFF = const(b"\x38") # Idle mode OFF
    IDMON = const(b"\x39") # Idle mode ON
    PIXSET = const(b"\x3A")  # COLMOD: Pixel format set
    RAMWRCON = const(b"\x3C")  # Write memory continue
    RAMRDCON = const(b"\x3E")  # Read memory continue
    WRDISBV = const(b"\x51")  # Write display brightness
    RDDISBV = const(b"\x52")  # Read display brightness value
    WRCTRLD = const(b"\x53")  # Write control display
    RDCTRLD = const(b"\x54")  # Read control display
    WRCABC = const(b"\x55")  # Write Content Adaptive Brightness control
    RDCABC = const(b"\x56")  # Read Content Adaptive Brightness control
    WRCABCMIN = const(b"\x5E")  # Write CABC Minimum Brightness (Write backlight control 1)
    RDCABCMIN = const(b"\x5F")  # Read CABC Minimum Brightness (Read backlight control 1)
    FRMCTR1 = const(b"\xB1")  # Frame rate control (In normal mode/full colors)
    FRMCTR2 = const(b"\xB2")  # Frame rate control (In idle mode/8 colors)
    FRMCTR3 = const(b"\xB3")  # Frame rate control (In partial mode/full colors)
    INVTR = const(b"\xB4")  # Display inversion control
    DISCTRL = const(b"\xB6")  # Display function control
    ETMOD = const(b"\xB7")  # Entry mode set
    PWCTRL1 = const(b"\xC0")  # Power control 1
    PWCTRL2 = const(b"\xC1")  # Power control 2
    VMCTRL1 = const(b"\xC5")  # VCOM control 1
    VMCTRL2 = const(b"\xC7")  # VCOM control 2
    PWCTRA = const(b"\xCB")  # Power control A
    PWCTRB = const(b"\xCF")  # Power control B
    RDID1 = const(b"\xDA")  # Read ID 1 (LCD module manufacturer ID)
    RDID2 = const(b"\xDB")  # Read ID 2 (Used to track LCD module driver version)
    RDID3 = const(b"\xDC")  # Read ID 3 (LCD module ID)
    RDID4 = const(b"\xDD")  # Read ID 4 (IC device code)
    PGAMCTRL = const(b"\xE0")  # Positive gamma correction
    NGAMCTRL = const(b"\xE1")  # Negative gamma correction
    DTIMCTA = const(b"\xE8")  # Driver timing control A
    DTIMCTB = const(b"\xEA")  # Driver timing control B
    PONSEQCT = const(b"\xED")  # Power on sequence control
    ENABLE3G = const(b"\xF2")  # Enable 3 gamma control
    IFCTL = const(b"\xF6")  # Interface control (16 bit data format selection)
    PUMPRC = const(b"\xF7")  # Pump ratio control
    
    def __init__(self, spi: SPI, cs: Pin, rst: Pin, dc: Pin,
                 width: int, height: int, bytes_max: int=50_000,
                 rotate_90_cw: bool=False, flip_h: bool=False, flip_v: bool=False,
                 color_mode=0, bgr: bool=False, gamma=True):
        """Initialize OLED.
        
        Args:
            spi (Class Spi): SPI interface for TFT
            cs (Class Pin): Chip select pin
            rst (Class Pin): Reset pin
            dc (Class Pin): Data/Command pin
            width (int): Screen width
            height (int): Screen height
            rotate_90_cw (Optional bool): Rotate the screen 90 degrees counter-clockwise. Also exchanges width and height.
            flip_h (Optional bool): Flip the screen horizontally (default False)
            flip_v (Optional bool): Flip the screen vertically (default False)
            color_mode (Optional int): Pixel color format (0 - RGB565, 1 - RGB888) (default 0)
            bgr (Optional bool): Swaps color bytes reading order (RGB -> BGR) (default False)
            gamma (Optional bool): Custom gamma correction (default True)
        """
        self.spi = spi
        self.cs = cs
        self.dc = dc
        self.rst = rst
        self.bgr = bgr
        self.color_mode = color_mode
        self.MAX_BYTES = const(bytes_max)
        
        # Take the required amount of color bytes according to the format.
        if color_mode == 0:
            self.color_bytes = 2
        elif color_mode == 1:
            self.color_bytes = 3
        
        self.rotation = 0
        if rotate_90_cw:
            self.rotation = 0b0010_0000
            self.width = height
            self.height = width
        else:
            self.rotation = 0b1000_0000
            self.width = width
            self.height = height
        
        self.rotation = self.rotation ^ flip_h << 7 ^ flip_v << 6 ^ bgr << 3  # Set the BGR bit

        # Initialize GPIO pins.
        self.cs.init(self.cs.OUT, value=1)
        self.dc.init(self.dc.OUT, value=0)
        self.rst.init(self.rst.OUT, value=1)
        self.reset()
        # Send initialization commands.
        self.write_cmd(self.SWRESET)  # Software reset
        sleep(.05)
        self.run_seq(
            (self.PWCTRB, [0x00, 0xC1, 0x30]),  # Power control B
            (self.PONSEQCT, [0x64, 0x03, 0x12, 0x81]),  # Power ON sequence control
            (self.DTIMCTA, [0x85, 0x00, 0x78]),  # Driver timing conrol A
            (self.PWCTRA, [0x39, 0x2C, 0x00, 0x34, 0x02]),  # Pwr control A
            (self.PUMPRC, [0x20]),  # Pump ratio control
            (self.DTIMCTB, [0x00, 0x00]),  # Driver timing control B
            (self.PWCTRL1, [0x23]),  # Power control 1
            (self.PWCTRL2, [0x10]),  # Power control 2
            (self.VMCTRL1, [0x3E, 0x28]),  # VCOM control 1
            (self.VMCTRL2, [0x86]),  # VCOM control 2
            (self.MADCTL, [self.rotation]),  # Memory access control
            (self.VSCRSADD, [0x00]),  # Vertical scrolling start address
            (self.PIXSET, [0x55 + self.color_mode]),  # COLMOD: Pixel format set
            (self.FRMCTR1, [0x00, 0x18]),  # Frame rate control
            (self.DISCTRL, [0x08, 0x82, 0x27]),  # Display function control
            (self.ENABLE3G, [0x00]),  # Enable 3 gamma control
            (self.GAMSET, [0x01])  # Gamma curve selected
        )
        if gamma:  # Use custom gamma correction values
            self.write_cmd(self.PGAMCTRL, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                           0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09,
                           0x00)
            self.write_cmd(self.NGAMCTRL, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                           0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36,
                           0x0F)
        self.write_cmd(self.SLPOUT)  # Exit sleep
        sleep(.05)
        self.write_cmd(self.DISPON)  # Display on
        sleep(.05)
        self.clear()
        gc.collect()
    
    def write_cmd(self, command: bytes, *args) -> None:
        """Write command to OLED.
        
        Args:
            command (byte): ILI9341 command code.
            *args (optional bytes): Data to transmit.
        """
        self.dc.value(0)
        self.cs.value(0)
        self.spi.write(command)
        self.cs.value(1)
        # Handle any passed data
        if len(args) > 0:
            self.write_data(bytes(args))
    
    def write_data(self, data) -> None:
        """Write data to OLED.

        Args:
            data (bytes): Data to transmit.
        """
        self.dc.value(1)
        self.cs.value(0)
        self.spi.write(data)
        self.cs.value(1)
    
    def run_seq(self, *seq: tuple(bytes, list(int)), select: bool=True, deselect: bool=True) -> None:
        """
        Write a sequence of commands and data to the display.
        
        Args:
            *seq: The commands and data to be executed.
                  The action structure is: tuple(command, [data_arguments]).
            select (bool): Select the chip at the start of the sequence (default True).
            deselect (bool): Deselect the chip at the end of the sequence (default True).
        """
        if select:
            self.cs.value(0)
        for command, data in seq:
            self.dc.value(0)
            self.spi.write(command)
            if data is not None:
                self.dc.value(1)
                self.spi.write(bytes(data))
        # end loop
        if deselect:
            self.cs.value(1)
    
    def read_data(self, command, buffer_size) -> bytes:
        self.dc.value(0)
        self.cs.value(0)
        self.spi.write(bytes([command]))
        self.dc.value(1)
        data = self.spi.read(buffer_size)
        self.cs.value(1)
        return data
    
    def set_block(self, x0, y0, x1, y1) -> None:
        """Set a block area to write to the display.
        
        Args:
            x0 (int):  Starting X position.
            y0 (int):  Starting Y position.
            x1 (int):  Ending X position.
            y1 (int):  Ending Y position.
        """
        self.run_seq((self.CASET, [x0 >> 8, x0 & 0xff, x1 >> 8, x1 & 0xff]),
                     (self.PASET, [y0 >> 8, y0 & 0xff, y1 >> 8, y1 & 0xff]),
                     (self.RAMWR, None))
    
    def block(self, x0: int, y0: int, x1: int, y1: int, data: bytes, select: bool=True, deselect: bool=True) -> None:
        """Write a block of data to display.

        Args:
            x0 (int):  Starting X position.
            y0 (int):  Starting Y position.
            x1 (int):  Ending X position.
            y1 (int):  Ending Y position.
            data (bytes): Data buffer to write.
            select (bool): Select the chip at the start of the sequence (default True).
            deselect (bool): Deselect the chip at the end of the sequence (default True).
        """
        self.run_seq((self.CASET, [x0 >> 8, x0 & 0xff, x1 >> 8, x1 & 0xff]),
                     (self.PASET, [y0 >> 8, y0 & 0xff, y1 >> 8, y1 & 0xff]),
                     (self.RAMWR, data),
                     select=select, deselect=deselect)
    
    def reset(self) -> None:
        """Perform reset: Low=initialization, High=normal operation.

        Notes: MicroPython implemntation
        """
        self.rst.value(0)
        sleep(.05)
        self.rst.value(1)
        sleep(.05)
    
    def cleanup(self) -> None:
        """Clean up resources."""
        self.clear()
        self.display_off()
        self.spi.deinit()
        print('display off')
    
    def clear(self, color: int=0x000000) -> None:
        """Fill the display with a chosen color.

        Args:
            color (int): RGB color value (Default: 0 = Black).
        Note:
            chunk_height is required to deal with memory allocation on some
            boards. Smaller values allocate less memory but take longer
            to execute.
            Higher values may result in memory allocation errors.
        """
        # Create the necessary variables for shorter a runtime.
        w: int = self.width
        h: int = self.height
        chunk_height: int = (self.MAX_BYTES // 3) // w
        final_chunk_height, remainder = divmod(h, chunk_height)
        # Prepare chunk.
        chunk = chunk_height * w * color.to_bytes(self.color_bytes, "big")
        
        # Set the clear area.
        self.run_seq((self.CASET, [0x0, 0x0, (w - 1) >> 8, (w - 1) & 0xff]),
                     (self.PASET, [0x0, 0x0, (h - 1) >> 8, (h - 1) & 0xff]),
                     (self.RAMWR, None),
                     deselect=False)
        # Set data transfer to display.
        self.dc.value(1)
        # Draw the chunks.
        for i in range(0, h, chunk_height):
            self.spi.write(chunk)
        # end loop
        # Draw the remaining chunk.
        if remainder:
            self.spi.write(color.to_bytes(self.color_bytes, "big") * remainder * final_chunk_height)
        
        self.cs(1)
        gc.collect()
    
    def display_off(self) -> None:
        """Turn display off."""
        self.write_cmd(self.DISPOFF)
    
    def display_on(self) -> None:
        """Turn display on."""
        self.write_cmd(self.DISPON)
    
    def draw_pixel(self, x: int, y: int, color: int, select: bool=True, deselect: bool=True) -> None:
        """Draw a single pixel.

        Args:
            x (int): X position.
            y (int): Y position.
            color (int): RGB color value.
            select (bool): Select the chip at the start of the sequence (default True).
            deselect (bool): Deselect the chip at the end of the sequence (default True).
        """
        self.run_seq((self.CASET, [x >> 8, x & 0xff, 0x0, 0x0]),
                     (self.PASET, [y >> 8, y & 0xff, 0x0, 0x0]),
                     (self.RAMWR, color.to_bytes(self.color_bytes, "big")),
                     select=select, deselect=deselect)
    
    def draw_image(self, path: str, x: int=0, y: int=0) -> None:
        # TODO: Rotate pictures
        # TODO: Handle large images
        """
        Draw image from a bmp file, but fast.
        Args:
            path: Image file path.
            x: X coordinate of image left.  Default is 0.
            y: Y coordinate of image top.  Default is 0.
        """
        # Get the image's data.
        offset, w, h, bpp, data_size, pb = self.get_image_properties(path)
        print(f"Offset: {offset}\nWidth: {w}\nHeight: {h}\nBits per pixel: {bpp}\nBitmap size: {data_size}\nPadding bytes: {pb}")
        
        with open(path, "rb") as f:
            f.seek(offset)
            for row in range(h):
                pixels = bytes(reversed(f.read(w * (bpp // 8))))
                self.block(x, y + row, x + w - 1, y + row, pixels)
                f.seek(pb, 1)
    
    def get_image_properties(self, path: str) -> tuple(int, int, int, int, int, int):
        """
        Gets a bmp image's properties.
        Args:
            path: Image file path.
        Returns:
            offset: Image's content start offset.
            w: Width of the image.
            h: Height of the image.
            bpp: Bits per pixel of the image.
            data_size: Size of the image's bitmap (including padding bytes).
            padding_bytes: The image's bitmap padding bytes.
        """
        with open(path, "rb") as f:
            file_signature = f.read(2)
            if file_signature != b"BM":
                raise TypeError("file format is not supported")
            # Seek to data start offset.
            f.seek(0xA)
            # Calculate the data start offset.
            offset = 0
            for i in range(4):
                offset += ord(f.read(1)) << (8 * i)
            # Seek width and height.
            f.seek(0x12)
            # Calculate the image's width.
            w = 0
            for i in range(4):
                w += ord(f.read(1)) << (8 * i)
            # Calculate the image's height.
            h = 0
            for i in range(4):
                h += ord(f.read(1)) << (8 * i)
            # Seek image's bits per pixel.
            f.seek(0x1C)
            bpp = 0
            for i in range(4):
                bpp += ord(f.read(1)) << (8 * i)
            if bpp // 8 != self.color_bytes:
                raise TypeError(f"the required file type is {self.color_bytes * 8}-bit color bitmap")
            # Seek bitmap size (including padding bytes).
            f.seek(0x22)
            # Calculate the bitmap size.
            data_size = 0
            for i in range(4):
                data_size += ord(f.read(1)) << (8 * i)
        # Calculate bitmap padding bytes.
        padding_bytes = (w * (bpp // 8)) % 4
        return offset, w, h, bpp, data_size, padding_bytes
    
    def draw_sprite(self, data: bytes, w: int, h: int, x: int=0, y: int=0) -> None:
        """
        Draw sprite from raw data.
        Args:
            data: Sprite pixel data.
            w: Width of the Sprite.
            h: Height of the Sprite.
            x: X coordinate of image left.  Default is 0.
            y: Y coordinate of image top.  Default is 0.
        """
        if len(data) <= self.MAX_BYTES:
            self.block(x, y, x + w - 1, y + h -1, data)
        else:
            chunk_height, remainder = divmod(self.MAX_BYTES, w * self.color_bytes)
            for i in range(y, y + h + 1, chunk_height):
                self.block(x, i, x + w - 1, i + chunk_height, data[i * w: w * (i + chunk_height)])
            # end loop
            self.block(x, y + h - remainder, x + w - 1, y + h, data[(h - remainder) * w: ])
    
    def load_sprite(self, path: str) -> tuple(bytes, int, int):
        """
        Load data from a bmp file to raw pixel data for a sprite.
        Args:
            path: Image file path.
        Returns:
            raw_data: The image's pixels.
            w: Width of the image.
            h: Height of the image.
        """
        with open(path, "rb") as f:
            # Get the header and info.
            header_and_info = f.read(0x36)
            # Make sure it's the correct file type
            if header_and_info[0:2] != b"BM":
                raise TypeError("file format is not supported")
            # Get file start offset.
            offset = sum(list(header_and_info[0xA + i] * (256 ** i) for i in range(4)))
            
            # Get the image's width and height.
            w = sum(list(header_and_info[0x12 + i] * (256 ** i) for i in range(4)))
            h = sum(list(header_and_info[0x16 + i] * (256 ** i) for i in range(4)))
            # Make sure the image is small enough.
            if w > self.width or h > self.height:
                raise ValueError("image is too big for the screen")
            
            data_size = sum(list(header_and_info[0x22 + i] * (256 ** i) for i in range(4)))
            
            # Get the image's bits per pixel.
            bpp = header_and_info[0x1C] + header_and_info[0x1D] * 256
            
            # Skip to the bitmap by reading what's left from the offset (palletes are ignored).
            f.seek(offset)
            
            # Make sure the color format fits the display color format
            if not bpp // 8 == self.color_bytes:
                raise TypeError(f"the required file type is {self.color_bytes * 8}-bit color bitmap")
            
            # Calculate the row padding bytes
            padding_bytes = (w * self.color_bytes) % 4
            
            # Draw the image chunk by chunk.
            raw_data = b""
            for b in range(0, data_size, w):
                curr = reversed(f.read(w * self.color_bytes))
                raw_data += bytes(curr)
                f.seek(padding_bytes, 1)
                gc.collect()
            # end loop
        data_size = len(raw_data)
        if data_size // w != data_size / w or data_size // h != data_size / h:
            raise UserWarning("the data doesn't fit it's dimensions, which could cause errors")
        
        return raw_data, w, h
    
    def draw_hline(self, x: int, y: int, l: int, color: int) -> None:
        """
        Draw a horizontal line (optimized for doing so).
        Args:
            x (int): Starting X position.
            Y (int): Starting Y position.
            l (int): Length of line.
            color (int): RGB color value for the line.
        """
        self.block(x, y ,x + l - 1, y, color.to_bytes(self.color_bytes) * l)
    
    def draw_vline(self, x: int, y: int, l: int, color: int) -> None:
        """
        Draw a vertical line (optimized for doing so).
        Args:
            x (int): Starting X position.
            Y (int): Starting Y position.
            l (int): Length of line.
            color (int): RGB color value for the line.
        """
        self.block(x, y ,x, y + l - 1, color.to_bytes(self.color_bytes) * l)
    
    def draw_line(self, x0: int, y0: int, x1: int, y1: int, color: int) -> None:
        """
        Draw a line that's enclosed within the screen area (optimized to do so)
        using a slightly modified Bresenham's algorithm.
        Note: Doesn't handle exceptions! make sure to pass valid values
              (smaller than screen's width and height and greater than 0).
              See draw_any_line to draw any line with any coordinates.
        Args:
            x0 (int): X starting position.
            y0 (int): Y starting position.
            x1 (int): X ending position.
            y1 (int): Y ending position.
            color (int): RGB color value for the line.
        """
        # Draw line optimally if possible.
        if x0 == x1:
            self.draw_vline(x0, y0 if y0 < y1 else y1, abs(y1 - y0), color)
            gc.collect()
            return
        elif y0 == y1:
            self.draw_hline(x0 if x0 < x1 else x1, y0, abs(x1 - x0), color)
            gc.collect()
            return
        # Create the line's constant variables.
        encoded_color: bytes = color.to_bytes(self.color_bytes, "big")
        m: float = (y1 - y0) / (x1 - x0)
        b: float = y0 - x0 * m
        # Exchange X and Y values if needed and prepare the drawing function accordingly.
        if abs(m) > 1:
            x0, y0 = y0, x0
            x1, y1 = y1, x1
            draw_segment = lambda end_x, end_y, w: self.block(end_y, end_x - w + 1, end_y, end_x, encoded_color * w, select=False,deselect=False)
        else:
            draw_segment = lambda end_x, end_y, w: self.block(end_x - w + 1, end_y, end_x, end_y, encoded_color * w, select=False,deselect=False)
        # Determine if the incrementation should occur at > 0 or < 0.
        if m > 0:
            reverse_check = -1
        else:
            reverse_check = 1
        # Exchange P0 and P1 if needed.
        if x0 > x1:
            y0, y1 = y1, y0
            x0, x1 = x1, x0
        # Recalculate differentials.
        m: float = (y1 - y0) / (x1 - x0)
        b: float = y0 - x0 * m
        # Create the evaluation function.
        D: callable = lambda x, y: m * x - y + b # = 0
        # Create iteration variables.
        y_step = 1 if y1 > y0 else -1
        yi = y0
        temp = 1
        # Draw the line.
        self.cs.value(0)
        for xi in range(x0, x1 + 1):
            if D(xi + 1, yi + 0.5 * y_step) * reverse_check < 0:
                draw_segment(xi, yi, temp)
                yi += y_step
                temp = 1
                continue
            temp += 1
        # end loop
        # Draw the remainder if there is.
        temp -= 1
        if temp:
            draw_segment(x1, yi, temp)
        # Deselect the display.
        self.cs.value(1)
        gc.collect()
    
    def draw_any_line(self, x0: int, y0: int, x1: int, y1: int, color: int) -> None:
        """
        Draw any line, be it outside of the screen, or whatever.
        Args:
            x0 (int): X starting position.
            y0 (int): Y starting position.
            x1 (int): X ending position.
            y1 (int): Y ending position.
            color (int): RGB color value for the line.
        """
#         def is_within_screen(x: int, y: int) -> bool:
#             """
#             Determine if a point will appear on screen.
#             Args:
#                 x (int): X coordinate position.
#                 y (int): Y coordinate position.
#             """
#             if 0 <= x < w and 0 <= y < h:
#                 return True
#             else:
#                 return False
        # Exchange x0 and x1 or y0 and y1 if neede.
        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0
        
        w = self.width
        h = self.height
        # Draw the line right away if the coordinates are valid.
        if 0 <= x0 < w and 0 <= x1 < w and 0 <= y0 < h and 0 <= y1 < h:
            self.draw_line(x0, y0, x1, y1, color)
            return
        # Get the constant variables.
        m: float = (y1 - y0) / (x1 - x0)
        b: float = y0 - x0 * m
        # Get the line's intersection points.
        P0 = (0, b) # Intersection at X = 0.
        P1 = (-1 * b / m, 0) # Intersection at Y = 0.
        P2 = (w, m * (w - 1) + b) # Intersection at X = w.
        P3 = ((h - 1 - b) / m, h) # Intersection at Y = h.
        # Find which points are valid.
        valid1 = None
        valid2 = None
        return None
    
    def fill_hrect(self, x, y, w, h, color):
        """Draw a filled rectangle (optimized for horizontal drawing).

        Args:
            x (int): Starting X position.
            y (int): Starting Y position.
            w (int): Width of rectangle.
            h (int): Height of rectangle.
            color (int): RGB565 color value.
        """
        if self.is_off_grid(x, y, x + w - 1, y + h - 1):
            return
        chunk_height = 1024 // w
        chunk_count, remainder = divmod(h, chunk_height)
        chunk_size = chunk_height * w
        chunk_y = y
        if chunk_count:
            buf = color.to_bytes(2, 'big') * chunk_size
            for c in range(0, chunk_count):
                self.block(x, chunk_y,
                           x + w - 1, chunk_y + chunk_height - 1,
                           buf)
                chunk_y += chunk_height

        if remainder:
            buf = color.to_bytes(2, 'big') * remainder * w
            self.block(x, chunk_y,
                       x + w - 1, chunk_y + remainder - 1,
                       buf)
    
    def draw_letter(self, x, y, letter, font, color, background=0,
                    landscape=False, rotate_180=False):
        """Draw a letter.

        Args:
            x (int): Starting X position.
            y (int): Starting Y position.
            letter (string): Letter to draw.
            font (XglcdFont object): Font.
            color (int): RGB565 color value.
            background (int): RGB565 background color (default: black)
            landscape (bool): Orientation (default: False = portrait)
            rotate_180 (bool): Rotate text by 180 degrees
        """
        buf, w, h = font.get_letter(letter, color, background, landscape)
        if rotate_180:
            # Manually rotate the buffer by 180 degrees
            # ensure bytes pairs for each pixel retain color565
            new_buf = bytearray(len(buf))
            num_pixels = len(buf) // 2
            for i in range(num_pixels):
                # The index for the new buffer's byte pair
                new_idx = (num_pixels - 1 - i) * 2
                # The index for the original buffer's byte pair
                old_idx = i * 2
                # Swap the pixels
                new_buf[new_idx], new_buf[new_idx + 1] = buf[old_idx], buf[old_idx + 1]
            buf = new_buf

        # Check for errors (Font could be missing specified letter)
        if w == 0:
            return w, h

        if landscape:
            y -= w
            if self.is_off_grid(x, y, x + h - 1, y + w - 1):
                return 0, 0
            self.block(x, y,
                       x + h - 1, y + w - 1,
                       buf)
        else:
            if self.is_off_grid(x, y, x + w - 1, y + h - 1):
                return 0, 0
            self.block(x, y,
                       x + w - 1, y + h - 1,
                       buf)
        return w, h
    
    def draw_text(self, x, y, text, font, color,  background=0,
                  landscape=False, rotate_180=False, spacing=1):
        """Draw text.

        Args:
            x (int): Starting X position
            y (int): Starting Y position
            text (string): Text to draw
            font (XglcdFont object): Font
            color (int): RGB565 color value
            background (int): RGB565 background color (default: black)
            landscape (bool): Orientation (default: False = portrait)
            rotate_180 (bool): Rotate text by 180 degrees
            spacing (int): Pixels between letters (default: 1)
        """
        iterable_text = reversed(text) if rotate_180 else text
        for letter in iterable_text:
            # Get letter array and letter dimensions
            w, h = self.draw_letter(x, y, letter, font, color, background,
                                    landscape, rotate_180)
            # Stop on error
            if w == 0 or h == 0:
                print('Invalid width {0} or height {1}'.format(w, h))
                return

            if landscape:
                # Fill in spacing
                if spacing:
                    self.fill_hrect(x, y - w - spacing, h, spacing, background)
                # Position y for next letter
                y -= (w + spacing)
            else:
                # Fill in spacing
                if spacing:
                    self.fill_hrect(x + w, y, spacing, h, background)
                # Position x for next letter
                x += (w + spacing)

                # # Fill in spacing
                # if spacing:
                #     self.fill_vrect(x + w, y, spacing, h, background)
                # # Position x for next letter
                # x += w + spacing
    
    def draw_text8x8(self, x, y, text, color,  background=0,
                     rotate=0):
        """Draw text using built-in MicroPython 8x8 bit font.

        Args:
            x (int): Starting X position.
            y (int): Starting Y position.
            text (string): Text to draw.
            color (int): RGB565 color value.
            background (int): RGB565 background color (default: black).
            rotate(int): 0, 90, 180, 270
        """
        w = len(text) * 8
        h = 8
        # Confirm coordinates in boundary
        if self.is_off_grid(x, y, x + 7, y + 7):
            return
        buf = bytearray(w * 16)
        fbuf = FrameBuffer(buf, w, h, RGB565)
        if background != 0:
            # Swap background color bytes to correct for framebuf endianness
            b_color = ((background & 0xFF) << 8) | ((background & 0xFF00) >> 8)
            fbuf.fill(b_color)
        # Swap text color bytes to correct for framebuf endianness
        t_color = ((color & 0xFF) << 8) | ((color & 0xFF00) >> 8)
        fbuf.text(text, 0, 0, t_color)
        if rotate == 0:
            self.block(x, y, x + w - 1, y + (h - 1), buf)
        elif rotate == 90:
            buf2 = bytearray(w * 16)
            fbuf2 = FrameBuffer(buf2, h, w, RGB565)
            for y1 in range(h):
                for x1 in range(w):
                    fbuf2.pixel(y1, x1,
                                fbuf.pixel(x1, (h - 1) - y1))
            self.block(x, y, x + (h - 1), y + w - 1, buf2)
        elif rotate == 180:
            buf2 = bytearray(w * 16)
            fbuf2 = FrameBuffer(buf2, w, h, RGB565)
            for y1 in range(h):
                for x1 in range(w):
                    fbuf2.pixel(x1, y1,
                                fbuf.pixel((w - 1) - x1, (h - 1) - y1))
            self.block(x, y, x + w - 1, y + (h - 1), buf2)
        elif rotate == 270:
            buf2 = bytearray(w * 16)
            fbuf2 = FrameBuffer(buf2, h, w, RGB565)
            for y1 in range(h):
                for x1 in range(w):
                    fbuf2.pixel(y1, x1,
                                fbuf.pixel((w - 1) - x1, y1))
            self.block(x, y, x + (h - 1), y + w - 1, buf2)
    
    def is_off_grid(self, xmin, ymin, xmax, ymax):
        """Check if coordinates extend past display boundaries.

        Args:
            xmin (int): Minimum horizontal pixel.
            ymin (int): Minimum vertical pixel.
            xmax (int): Maximum horizontal pixel.
            ymax (int): Maximum vertical pixel.
        Returns:
            boolean: False = Coordinates OK, True = Error.
        """
        if xmin < 0:
            print('x-coordinate: {0} below minimum of 0.'.format(xmin))
            return True
        if ymin < 0:
            print('y-coordinate: {0} below minimum of 0.'.format(ymin))
            return True
        if xmax >= self.width:
            print('x-coordinate: {0} above maximum of {1}.'.format(
                xmax, self.width - 1))
            return True
        if ymax >= self.height:
            print('y-coordinate: {0} above maximum of {1}.'.format(
                ymax, self.height - 1))
            return True
        return False
    
    def scroll(self, y):
        """Scroll display vertically.

        Args:
            y (int): Number of pixels to scroll display.
        """
        self.write_cmd(self.VSCRSADD, y >> 8, y & 0xFF)
    
    def set_scroll(self, top, bottom):
        """Set the height of the top and bottom scroll margins.

        Args:
            top (int): Height of top scroll margin
            bottom (int): Height of bottom scroll margin
        """
        if top + bottom <= self.height:
            middle = self.height - (top + bottom)
            self.write_cmd(self.VSCRDEF,
                           top >> 8,
                           top & 0xFF,
                           middle >> 8,
                           middle & 0xFF,
                           bottom >> 8,
                           bottom & 0xFF)
    
    def sleep(self, enable=True):
        """Enters or exits sleep mode.

        Args:
            enable (bool): True (default)=Enter sleep mode, False=Exit sleep
        """
        if enable:
            self.write_cmd(self.SLPIN)
        else:
            self.write_cmd(self.SLPOUT)
    
