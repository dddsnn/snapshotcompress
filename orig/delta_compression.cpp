/* 
    Delta Compression by Glenn Fiedler. 
    This source code is placed in the public domain.
    http://gafferongames.com/2015/03/14/the-networked-physics-data-compression-challenge/
*/

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>

static const int MaxContexts = 8;
static const int NumCubes = 901;
static const int MaxPacketSize = 4 * 1024;
static const int UnitsPerMeter = 512;
static const int OrientationBits = 9;
static const int PositionBoundXY = 32;
static const int PositionBoundZ = 16;
static const int QuantizedPositionBoundXY = UnitsPerMeter * PositionBoundXY - 1;
static const int QuantizedPositionBoundZ = UnitsPerMeter * PositionBoundZ - 1;

template <uint32_t x> struct PopCount
{
    enum {   a = x - ( ( x >> 1 )       & 0x55555555 ),
             b =   ( ( ( a >> 2 )       & 0x33333333 ) + ( a & 0x33333333 ) ),
             c =   ( ( ( b >> 4 ) + b ) & 0x0f0f0f0f ),
             d =   c + ( c >> 8 ),
             e =   d + ( d >> 16 ),

        result = e & 0x0000003f 
    };
};

template <uint32_t x> struct Log2
{
    enum {   a = x | ( x >> 1 ),
             b = a | ( a >> 2 ),
             c = b | ( b >> 4 ),
             d = c | ( c >> 8 ),
             e = d | ( d >> 16 ),
             f = e >> 1,

        result = PopCount<f>::result
    };
};

template <int64_t min, int64_t max> struct BitsRequired
{
    static const uint32_t result = ( min == max ) ? 0 : Log2<uint32_t(max-min)>::result + 1;
};

inline uint32_t popcount( uint32_t x )
{
    const uint32_t a = x - ( ( x >> 1 )       & 0x55555555 );
    const uint32_t b =   ( ( ( a >> 2 )       & 0x33333333 ) + ( a & 0x33333333 ) );
    const uint32_t c =   ( ( ( b >> 4 ) + b ) & 0x0f0f0f0f );
    const uint32_t d =   c + ( c >> 8 );
    const uint32_t e =   d + ( d >> 16 );
    const uint32_t result = e & 0x0000003f;
    return result;
}

#ifdef __GNUC__

inline int bits_required( uint32_t min, uint32_t max )
{
    return 32 - __builtin_clz( max - min );
}

#else

inline uint32_t log2( uint32_t x )
{
    const uint32_t a = x | ( x >> 1 );
    const uint32_t b = a | ( a >> 2 );
    const uint32_t c = b | ( b >> 4 );
    const uint32_t d = c | ( c >> 8 );
    const uint32_t e = d | ( d >> 16 );
    const uint32_t f = e >> 1;
    return popcount( f );
}

inline int bits_required( uint32_t min, uint32_t max )
{
    return ( min == max ) ? 0 : log2( max-min ) + 1;
}

#endif

template <typename T> const T & min( const T & a, const T & b )
{
    return ( a < b ) ? a : b;
}

template <typename T> const T & max( const T & a, const T & b )
{
    return ( a > b ) ? a : b;
}

template <typename T> T clamp( const T & value, const T & min, const T & max )
{
    if ( value < min )
        return min;
    else if ( value > max )
        return max;
    else
        return value;
}

template <typename T> void swap( T & a, T & b )
{
    T tmp = a;
    a = b;
    b = tmp;
};

template <typename T> T abs( const T & value )
{
    return ( value < 0 ) ? -value : value;
}

#define CPU_LITTLE_ENDIAN 1
#define CPU_BIG_ENDIAN 2

#if    defined(__386__) || defined(i386)    || defined(__i386__)  \
    || defined(__X86)   || defined(_M_IX86)                       \
    || defined(_M_X64)  || defined(__x86_64__)                    \
    || defined(alpha)   || defined(__alpha) || defined(__alpha__) \
    || defined(_M_ALPHA)                                          \
    || defined(ARM)     || defined(_ARM)    || defined(__arm__)   \
    || defined(WIN32)   || defined(_WIN32)  || defined(__WIN32__) \
    || defined(_WIN32_WCE) || defined(__NT__)                     \
    || defined(__MIPSEL__)
  #define CPU_ENDIAN CPU_LITTLE_ENDIAN
#else
  #define CPU_ENDIAN CPU_BIG_ENDIAN
#endif

inline uint32_t host_to_network( uint32_t value )
{
#if CPU_ENDIAN == CPU_BIG_ENDIAN
    return __builtin_bswap32( value );
#else
    return value;
#endif
}

inline uint32_t network_to_host( uint32_t value )
{
#if CPU_ENDIAN == CPU_BIG_ENDIAN
    return __builtin_bswap32( value );
#else
    return value;
#endif
}

class BitWriter
{
public:

    BitWriter( void * data, int bytes ) : m_data( (uint32_t*)data ), m_numWords( bytes / 4 )
    {
        assert( data );
        assert( ( bytes % 4 ) == 0 );           // IMPORTANT: buffer size must be a multiple of four!
        m_numBits = m_numWords * 32;
        m_bitsWritten = 0;
        m_scratch = 0;
        m_bitIndex = 0;
        m_wordIndex = 0;
        m_overflow = false;
        memset( m_data, 0, bytes );
    }

    void WriteBits( uint32_t value, int bits )
    {
        assert( bits > 0 );
        assert( bits <= 32 );
        assert( m_bitsWritten + bits <= m_numBits );

        if ( m_bitsWritten + bits > m_numBits )
        {
            m_overflow = true;
            return;
        }

        value &= ( uint64_t( 1 ) << bits ) - 1;

        m_scratch |= uint64_t( value ) << ( 64 - m_bitIndex - bits );

        m_bitIndex += bits;

        if ( m_bitIndex >= 32 )
        {
            assert( m_wordIndex < m_numWords );
            m_data[m_wordIndex] = host_to_network( uint32_t( m_scratch >> 32 ) );
            m_scratch <<= 32;
            m_bitIndex -= 32;
            m_wordIndex++;
        }

        m_bitsWritten += bits;
    }

    void WriteAlign()
    {
        const int remainderBits = m_bitsWritten % 8;
        if ( remainderBits != 0 )
        {
            uint32_t zero = 0;
            WriteBits( zero, 8 - remainderBits );
            assert( m_bitsWritten % 8 == 0 );
        }
    }

    void WriteBytes( const uint8_t * data, int bytes )
    {
        assert( GetAlignBits() == 0 );
        if ( m_bitsWritten + bytes * 8 >= m_numBits )
        {
            m_overflow = true;
            return;
        }

        assert( m_bitIndex == 0 || m_bitIndex == 8 || m_bitIndex == 16 || m_bitIndex == 24 );

        int headBytes = ( 4 - m_bitIndex / 8 ) % 4;
        if ( headBytes > bytes )
            headBytes = bytes;
        for ( int i = 0; i < headBytes; ++i )
            WriteBits( data[i], 8 );
        if ( headBytes == bytes )
            return;

        assert( GetAlignBits() == 0 );

        int numWords = ( bytes - headBytes ) / 4;
        if ( numWords > 0 )
        {
            assert( m_bitIndex == 0 );
            memcpy( &m_data[m_wordIndex], data + headBytes, numWords * 4 );
            m_bitsWritten += numWords * 32;
            m_wordIndex += numWords;
            m_scratch = 0;
        }

        assert( GetAlignBits() == 0 );

        int tailStart = headBytes + numWords * 4;
        int tailBytes = bytes - tailStart;
        assert( tailBytes >= 0 && tailBytes < 4 );
        for ( int i = 0; i < tailBytes; ++i )
            WriteBits( data[tailStart+i], 8 );

        assert( GetAlignBits() == 0 );

        assert( headBytes + numWords * 4 + tailBytes == bytes );
    }

    void FlushBits()
    {
        if ( m_bitIndex != 0 )
        {
            assert( m_wordIndex < m_numWords );
            if ( m_wordIndex >= m_numWords )
            {
                m_overflow = true;
                return;
            }
            m_data[m_wordIndex++] = host_to_network( uint32_t( m_scratch >> 32 ) );
        }
    }

    int GetAlignBits() const
    {
        return ( 8 - m_bitsWritten % 8 ) % 8;
    }

    int GetBitsWritten() const
    {
        return m_bitsWritten;
    }

    int GetBitsAvailable() const
    {
        return m_numBits - m_bitsWritten;
    }

    const uint8_t * GetData() const
    {
        return (uint8_t*) m_data;
    }

    int GetBytesWritten() const
    {
        return m_wordIndex * 4;
    }

    int GetTotalBytes() const
    {
        return m_numWords * 4;
    }

    bool IsOverflow() const
    {
        return m_overflow;
    }

private:

    uint32_t * m_data;
    uint64_t m_scratch;
    int m_numBits;
    int m_numWords;
    int m_bitsWritten;
    int m_bitIndex;
    int m_wordIndex;
    bool m_overflow;
};

class BitReader
{
public:

    BitReader( const void * data, int bytes ) : m_data( (const uint32_t*)data ), m_numWords( bytes / 4 )
    {
        assert( data );
        assert( ( bytes % 4 ) == 0 );           // IMPORTANT: buffer size must be a multiple of four!
        m_numBits = m_numWords * 32;
        m_bitsRead = 0;
        m_bitIndex = 0;
        m_wordIndex = 0;
        m_scratch = network_to_host( m_data[0] );
        m_overflow = false;
    }

    uint32_t ReadBits( int bits )
    {
        assert( bits > 0 );
        assert( bits <= 32 );
        assert( m_bitsRead + bits <= m_numBits );

        if ( m_bitsRead + bits > m_numBits )
        {
            m_overflow = true;
            return 0;
        }

        m_bitsRead += bits;

        assert( m_bitIndex < 32 );

        if ( m_bitIndex + bits < 32 )
        {
            m_scratch <<= bits;
            m_bitIndex += bits;
        }
        else
        {
            m_wordIndex++;
            assert( m_wordIndex < m_numWords );
            const uint32_t a = 32 - m_bitIndex;
            const uint32_t b = bits - a;
            m_scratch <<= a;
            m_scratch |= network_to_host( m_data[m_wordIndex] );
            m_scratch <<= b;
            m_bitIndex = b;
        }

        const uint32_t output = uint32_t( m_scratch >> 32 );

        m_scratch &= 0xFFFFFFFF;

        return output;
    }

    void ReadAlign()
    {
        const int remainderBits = m_bitsRead % 8;
        if ( remainderBits != 0 )
        {
            #ifdef NDEBUG
            ReadBits( 8 - remainderBits );
            #else
            uint32_t value = ReadBits( 8 - remainderBits );
            assert( value == 0 );
            assert( m_bitsRead % 8 == 0 );
            #endif
        }
    }

    void ReadBytes( uint8_t * data, int bytes )
    {
        assert( GetAlignBits() == 0 );

        if ( m_bitsRead + bytes * 8 >= m_numBits )
        {
            memset( data, bytes, 0 );
            m_overflow = true;
            return;
        }

        assert( m_bitIndex == 0 || m_bitIndex == 8 || m_bitIndex == 16 || m_bitIndex == 24 );

        int headBytes = ( 4 - m_bitIndex / 8 ) % 4;
        if ( headBytes > bytes )
            headBytes = bytes;
        for ( int i = 0; i < headBytes; ++i )
            data[i] = ReadBits( 8 );
        if ( headBytes == bytes )
            return;

        assert( GetAlignBits() == 0 );

        int numWords = ( bytes - headBytes ) / 4;
        if ( numWords > 0 )
        {
            assert( m_bitIndex == 0 );
            memcpy( data + headBytes, &m_data[m_wordIndex], numWords * 4 );
            m_bitsRead += numWords * 32;
            m_wordIndex += numWords;
            m_scratch = network_to_host( m_data[m_wordIndex] );
        }

        assert( GetAlignBits() == 0 );

        int tailStart = headBytes + numWords * 4;
        int tailBytes = bytes - tailStart;
        assert( tailBytes >= 0 && tailBytes < 4 );
        for ( int i = 0; i < tailBytes; ++i )
            data[tailStart+i] = ReadBits( 8 );

        assert( GetAlignBits() == 0 );

        assert( headBytes + numWords * 4 + tailBytes == bytes );
    }

    int GetAlignBits() const
    {
        return ( 8 - m_bitsRead % 8 ) % 8;
    }

    int GetBitsRead() const
    {
        return m_bitsRead;
    }

    int GetBytesRead() const
    {
        return ( m_wordIndex + 1 ) * 4;
    }

    int GetBitsRemaining() const
    {
        return m_numBits - m_bitsRead;
    }

    int GetTotalBits() const 
    {
        return m_numBits;
    }

    int GetTotalBytes() const
    {
        return m_numBits * 8;
    }

    bool IsOverflow() const
    {
        return m_overflow;
    }

private:

    const uint32_t * m_data;
    uint64_t m_scratch;
    int m_numBits;
    int m_numWords;
    int m_bitsRead;
    int m_bitIndex;
    int m_wordIndex;
    bool m_overflow;
};

class WriteStream
{
public:

    enum { IsWriting = 1 };
    enum { IsReading = 0 };

    WriteStream( uint8_t * buffer, int bytes ) : m_writer( buffer, bytes ), m_context( nullptr ), m_aborted( false ) {}

    void SerializeInteger( int32_t value, int32_t min, int32_t max )
    {
        assert( min < max );
        assert( value >= min );
        assert( value <= max );
        const int bits = bits_required( min, max );
        uint32_t unsigned_value = value - min;
        m_writer.WriteBits( unsigned_value, bits );
    }

    void SerializeBits( uint32_t value, int bits )
    {
        assert( bits > 0 );
        assert( bits <= 32 );
        m_writer.WriteBits( value, bits );
    }

    void SerializeBytes( const uint8_t * data, int bytes )
    {
        Align();
        m_writer.WriteBytes( data, bytes );
    }

    void Align()
    {
        m_writer.WriteAlign();
    }

    int GetAlignBits() const
    {
        return m_writer.GetAlignBits();
    }

    bool Check( uint32_t magic )
    {
        Align();
        SerializeBits( magic, 32 );
        return true;
    }

    void Flush()
    {
        m_writer.FlushBits();
    }

    const uint8_t * GetData() const
    {
        return m_writer.GetData();
    }

    int GetBytesProcessed() const
    {
        return m_writer.GetBytesWritten();
    }

    int GetBitsProcessed() const
    {
        return m_writer.GetBitsWritten();
    }

    int GetBitsRemaining() const
    {
        return GetTotalBits() - GetBitsProcessed();
    }

    int GetTotalBits() const
    {
        return m_writer.GetTotalBytes() * 8;
    }

    int GetTotalBytes() const
    {
        return m_writer.GetTotalBytes();
    }

    bool IsOverflow() const
    {
        return m_writer.IsOverflow();
    }

    void SetContext( const void ** context )
    {
        m_context = context;
    }

    const void * GetContext( int index ) const
    {
        assert( index >= 0 );
        assert( index < MaxContexts );
        return m_context ? m_context[index] : nullptr;
    }

    void Abort()
    {
        m_aborted = true;
    }

    bool Aborted() const
    {
        return m_aborted;
    }

private:

    BitWriter m_writer;
    const void ** m_context;
    bool m_aborted;
};

class ReadStream
{
public:

    enum { IsWriting = 0 };
    enum { IsReading = 1 };

    ReadStream( uint8_t * buffer, int bytes ) : m_bitsRead(0), m_reader( buffer, bytes ), m_context( nullptr ), m_aborted( false ) {}

    void SerializeInteger( int32_t & value, int32_t min, int32_t max )
    {
        assert( min < max );
        const int bits = bits_required( min, max );
        uint32_t unsigned_value = m_reader.ReadBits( bits );
        value = (int32_t) unsigned_value + min;
        m_bitsRead += bits;
    }

    void SerializeBits( uint32_t & value, int bits )
    {
        assert( bits > 0 );
        assert( bits <= 32 );
        uint32_t read_value = m_reader.ReadBits( bits );
        value = read_value;
        m_bitsRead += bits;
    }

    void SerializeBytes( uint8_t * data, int bytes )
    {
        Align();
        m_reader.ReadBytes( data, bytes );
        m_bitsRead += bytes * 8;
    }

    void Align()
    {
        m_reader.ReadAlign();
    }

    int GetAlignBits() const
    {
        return m_reader.GetAlignBits();
    }

    bool Check( uint32_t magic )
    {
        Align();
        uint32_t value = 0;
        SerializeBits( value, 32 );
        assert( value == magic );
        return value == magic;
    }

    int GetBitsProcessed() const
    {
        return m_bitsRead;
    }

    int GetBytesProcessed() const
    {
        return m_bitsRead / 8 + ( m_bitsRead % 8 ? 1 : 0 );
    }

    bool IsOverflow() const
    {
        return m_reader.IsOverflow();
    }

    void SetContext( const void ** context )
    {
        m_context = context;
    }

    const void * GetContext( int index ) const
    {
        assert( index >= 0 );
        assert( index < MaxContexts );
        return m_context ? m_context[index] : nullptr;
    }

    void Abort()
    {
        m_aborted = true;
    }

    bool Aborted() const
    {
        return m_aborted;
    }

    int GetBytesRead() const
    {
        return m_reader.GetBytesRead();
    }

private:

    int m_bitsRead;
    BitReader m_reader;
    const void ** m_context;
    bool m_aborted;
};

class MeasureStream
{
public:

    enum { IsWriting = 1 };
    enum { IsReading = 0 };

    MeasureStream( int bytes ) : m_totalBytes( bytes ), m_bitsWritten(0), m_context( nullptr ), m_aborted( false ) {}

    void SerializeInteger( int32_t value, int32_t min, int32_t max )
    {
        assert( min < max );
        assert( value >= min );
        assert( value <= max );
        const int bits = bits_required( min, max );
        m_bitsWritten += bits;
    }

    void SerializeBits( uint32_t value, int bits )
    {
        assert( bits > 0 );
        assert( bits <= 32 );
        m_bitsWritten += bits;
    }

    void SerializeBytes( const uint8_t * data, int bytes )
    {
        Align();
        m_bitsWritten += bytes * 8;
    }

    void Align()
    {
        const int alignBits = GetAlignBits();
        m_bitsWritten += alignBits;
    }

    int GetAlignBits() const
    {
        return 7;       // worst case
    }

    bool Check( uint32_t magic )
    {
        Align();
        m_bitsWritten += 32;
        return true;
    }

    int GetBitsProcessed() const
    {
        return m_bitsWritten;
    }

    int GetBytesProcessed() const
    {
        return m_bitsWritten / 8 + ( m_bitsWritten % 8 ? 1 : 0 );
    }

    int GetTotalBytes() const
    {
        return m_totalBytes;
    }

    int GetTotalBits() const
    {
        return m_totalBytes * 8;
    }

    bool IsOverflow() const
    {
        return m_bitsWritten > m_totalBytes * 8;
    }

    void SetContext( const void ** context )
    {
        m_context = context;
    }

    const void * GetContext( int index ) const
    {
        assert( index >= 0 );
        assert( index < MaxContexts );
        return m_context ? m_context[index] : nullptr;
    }

    void Abort()
    {
        m_aborted = true;
    }

    bool Aborted() const
    {
        return m_aborted;
    }

private:

    int m_totalBytes;
    int m_bitsWritten;
    const void ** m_context;
    bool m_aborted;
};

template <typename T> void serialize_object( ReadStream & stream, T & object )
{                        
    object.SerializeRead( stream );
}

template <typename T> void serialize_object( WriteStream & stream, T & object )
{                        
    object.SerializeWrite( stream );
}

template <typename T> void serialize_object( MeasureStream & stream, T & object )
{                        
    object.SerializeMeasure( stream );
}

#define serialize_int( stream, value, min, max )            \
    do                                                      \
    {                                                       \
        assert( min < max );                                \
        int32_t int32_value;                                \
        if ( Stream::IsWriting )                            \
        {                                                   \
            assert( value >= min );                         \
            assert( value <= max );                         \
            int32_value = (int32_t) value;                  \
        }                                                   \
        stream.SerializeInteger( int32_value, min, max );   \
        if ( Stream::IsReading )                            \
        {                                                   \
            value = (decltype(value)) int32_value;          \
            assert( value >= min );                         \
            assert( value <= max );                         \
        }                                                   \
    } while (0)

#define serialize_bits( stream, value, bits )               \
    do                                                      \
    {                                                       \
        assert( bits > 0 );                                 \
        assert( bits <= 32 );                               \
        uint32_t uint32_value;                              \
        if ( Stream::IsWriting )                            \
            uint32_value = (uint32_t) value;                \
        stream.SerializeBits( uint32_value, bits );         \
        if ( Stream::IsReading )                            \
            value = (decltype(value)) uint32_value;         \
    } while (0)

#define serialize_bool( stream, value ) serialize_bits( stream, value, 1 )

template <typename Stream> void serialize_uint16( Stream & stream, uint16_t & value )
{
    serialize_bits( stream, value, 16 );
}

template <typename Stream> void serialize_uint32( Stream & stream, uint32_t & value )
{
    serialize_bits( stream, value, 32 );
}

template <typename Stream> void serialize_uint64( Stream & stream, uint64_t & value )
{
    uint32_t hi,lo;
    if ( Stream::IsWriting )
    {
        lo = value & 0xFFFFFFFF;
        hi = value >> 32;
    }
    serialize_bits( stream, lo, 32 );
    serialize_bits( stream, hi, 32 );
    if ( Stream::IsReading )
        value = ( uint64_t(hi) << 32 ) | lo;
}

template <typename Stream> void serialize_int16( Stream & stream, int16_t & value )
{
    serialize_bits( stream, value, 16 );
}

template <typename Stream> void serialize_int32( Stream & stream, int32_t & value )
{
    serialize_bits( stream, value, 32 );
}

template <typename Stream> void serialize_int64( Stream & stream, int64_t & value )
{
    uint32_t hi,lo;
    if ( Stream::IsWriting )
    {
        lo = uint64_t(value) & 0xFFFFFFFF;
        hi = uint64_t(value) >> 32;
    }
    serialize_bits( stream, lo, 32 );
    serialize_bits( stream, hi, 32 );
    if ( Stream::IsReading )
        value = ( int64_t(hi) << 32 ) | lo;
}

template <typename Stream> void serialize_float( Stream & stream, float & value )
{
    union FloatInt
    {
        float float_value;
        uint32_t int_value;
    };

    FloatInt tmp;
    if ( Stream::IsWriting )
        tmp.float_value = value;

    serialize_uint32( stream, tmp.int_value );

    if ( Stream::IsReading )
        value = tmp.float_value;
}

template <typename Stream> inline void internal_serialize_float( Stream & stream, float & value, float min, float max, float res )
{
    const float delta = max - min;
    const float values = delta / res;
    const uint32_t maxIntegerValue = (uint32_t) ceil( values );
    const int bits = bits_required( 0, maxIntegerValue );
    
    uint32_t integerValue = 0;
    
    if ( Stream::IsWriting )
    {
        float normalizedValue = clamp( ( value - min ) / delta, 0.0f, 1.0f );
        integerValue = (uint32_t) floor( normalizedValue * maxIntegerValue + 0.5f );
    }
    
    stream.SerializeBits( integerValue, bits );

    if ( Stream::IsReading )
    {
        const float normalizedValue = integerValue / float( maxIntegerValue );
        value = normalizedValue * delta + min;
    }
}

#define serialize_compressed_float( stream, value, min, max, res )                                        \
do                                                                                                        \
{                                                                                                         \
    internal_serialize_float( stream, value, min, max, res );                                             \
}                                                                                                         \
while(0)

template <typename Stream> void serialize_double( Stream & stream, double & value )
{
    union DoubleInt
    {
        double double_value;
        uint64_t int_value;
    };

    DoubleInt tmp;
    if ( Stream::IsWriting )
        tmp.double_value = value;

    serialize_uint64( stream, tmp.int_value );

    if ( Stream::IsReading )
        value = tmp.double_value;
}

template <typename Stream> void serialize_bytes( Stream & stream, uint8_t * data, int bytes )
{
    stream.SerializeBytes( data, bytes );        
}

template <typename Stream> void serialize_string( Stream & stream, char * string, int buffer_size )
{
    uint32_t length;
    if ( Stream::IsWriting )
        length = strlen( string );
    stream.Align();
    stream.SerializeBits( length, 32 );
    assert( length < buffer_size - 1 );
    stream.SerializeBytes( (uint8_t*)string, length );
    if ( Stream::IsReading )
        string[length] = '\0';
}

template <typename Stream> bool serialize_check( Stream & stream, uint32_t magic )
{
    return stream.Check( magic );
}

#define SERIALIZE_OBJECT( stream )                                                          \
    void SerializeRead( class ReadStream & stream ) { Serialize( stream ); };               \
    void SerializeWrite( class WriteStream & stream ) { Serialize( stream ); };             \
    void SerializeMeasure( class MeasureStream & stream ) { Serialize( stream ); };         \
    template <typename Stream> void Serialize( Stream & stream )

template <int bits> struct compressed_quaternion
{
    enum { max_value = (1<<bits)-1 };

    uint32_t largest : 2;
    uint32_t integer_a : bits;
    uint32_t integer_b : bits;
    uint32_t integer_c : bits;

    void Load( float x, float y, float z, float w )
    {
        assert( bits > 1 );
        assert( bits <= 10 );

        const float minimum = - 1.0f / 1.414214f;       // 1.0f / sqrt(2)
        const float maximum = + 1.0f / 1.414214f;

        const float scale = float( ( 1 << bits ) - 1 );

        const float abs_x = fabs( x );
        const float abs_y = fabs( y );
        const float abs_z = fabs( z );
        const float abs_w = fabs( w );

        largest = 0;
        float largest_value = abs_x;

        if ( abs_y > largest_value )
        {
            largest = 1;
            largest_value = abs_y;
        }

        if ( abs_z > largest_value )
        {
            largest = 2;
            largest_value = abs_z;
        }

        if ( abs_w > largest_value )
        {
            largest = 3;
            largest_value = abs_w;
        }

        float a = 0;
        float b = 0;
        float c = 0;

        switch ( largest )
        {
            case 0:
                if ( x >= 0 )
                {
                    a = y;
                    b = z;
                    c = w;
                }
                else
                {
                    a = -y;
                    b = -z;
                    c = -w;
                }
                break;

            case 1:
                if ( y >= 0 )
                {
                    a = x;
                    b = z;
                    c = w;
                }
                else
                {
                    a = -x;
                    b = -z;
                    c = -w;
                }
                break;

            case 2:
                if ( z >= 0 )
                {
                    a = x;
                    b = y;
                    c = w;
                }
                else
                {
                    a = -x;
                    b = -y;
                    c = -w;
                }
                break;

            case 3:
                if ( w >= 0 )
                {
                    a = x;
                    b = y;
                    c = z;
                }
                else
                {
                    a = -x;
                    b = -y;
                    c = -z;
                }
                break;

            default:
                assert( false );
        }

        const float normal_a = ( a - minimum ) / ( maximum - minimum ); 
        const float normal_b = ( b - minimum ) / ( maximum - minimum );
        const float normal_c = ( c - minimum ) / ( maximum - minimum );

        integer_a = floor( normal_a * scale + 0.5f );
        integer_b = floor( normal_b * scale + 0.5f );
        integer_c = floor( normal_c * scale + 0.5f );
    }

    void Save( float & x, float & y, float & z, float & w ) const
    {
        // note: you're going to want to normalize the quaternion returned from this function

        assert( bits > 1 );
        assert( bits <= 10 );

        const float minimum = - 1.0f / 1.414214f;       // 1.0f / sqrt(2)
        const float maximum = + 1.0f / 1.414214f;

        const float scale = float( ( 1 << bits ) - 1 );

        const float inverse_scale = 1.0f / scale;

        const float a = integer_a * inverse_scale * ( maximum - minimum ) + minimum;
        const float b = integer_b * inverse_scale * ( maximum - minimum ) + minimum;
        const float c = integer_c * inverse_scale * ( maximum - minimum ) + minimum;

        switch ( largest )
        {
            case 0:
            {
                x = sqrtf( 1 - a*a - b*b - c*c );
                y = a;
                z = b;
                w = c;
            }
            break;

            case 1:
            {
                x = a;
                y = sqrtf( 1 - a*a - b*b - c*c );
                z = b;
                w = c;
            }
            break;

            case 2:
            {
                x = a;
                y = b;
                z = sqrtf( 1 - a*a - b*b - c*c );
                w = c;
            }
            break;

            case 3:
            {
                x = a;
                y = b;
                z = c;
                w = sqrtf( 1 - a*a - b*b - c*c );
            }
            break;

            default:
            {
                assert( false );
                x = 0;
                y = 0;
                z = 0;
                w = 1;
            }
        }
    }

    SERIALIZE_OBJECT( stream )
    {
        serialize_bits( stream, largest, 2 );
        serialize_bits( stream, integer_a, bits );
        serialize_bits( stream, integer_b, bits );
        serialize_bits( stream, integer_c, bits );
    }

    bool operator == ( const compressed_quaternion & other ) const
    {
        if ( largest != other.largest )
            return false;

        if ( integer_a != other.integer_a )
            return false;

        if ( integer_b != other.integer_b )
            return false;

        if ( integer_c != other.integer_c )
            return false;

        return true;
    }

    bool operator != ( const compressed_quaternion & other ) const
    {
        return ! ( *this == other );
    }
};

inline int count_relative_index_bits( bool * changed )
{
    int bits = 8;           // 0..255 num changed
    bool first = true;
    int previous_index = 0;

    for ( int i = 0; i < NumCubes; ++i )
    {
        if ( !changed[i] )
            continue;

        if ( first )
        {
            bits += 10;
            first = false;
            previous_index = i;
        }
        else
        {
            const int difference = i - previous_index;

            if ( difference == 1 )
            {
                bits += 1;
            }
            else if ( difference <= 6 )
            {
                bits += 1 + 1 + 2;
            }
            else if ( difference <= 14 )
            {
                bits += 1 + 1 + 1 + 3;
            }
            else if ( difference <= 30 )
            {
                bits += 1 + 1 + 1 + 1 + 4;
            }
            else if ( difference <= 62 )
            {
                bits += 1 + 1 + 1 + 1 + 1 + 5;
            }
            else if ( difference <= 126 )
            {
                bits += 1 + 1 + 1 + 1 + 1 + 1 + 6;
            }
            else
            {
                bits += 1 + 1 + 1 + 1 + 1 + 1 + 1 + 10;
            }

            previous_index = i;
        }
    }

    return bits;
}

template <typename Stream> void serialize_relative_index( Stream & stream, int previous, int & current )
{
    uint32_t difference;
    if ( Stream::IsWriting )
    {
        assert( previous < current );
        difference = current - previous;
        assert( difference > 0 );
    }

    // +1 (1 bit)

    bool plusOne;
    if ( Stream::IsWriting )
        plusOne = difference == 1;
    serialize_bool( stream, plusOne );
    if ( plusOne )
    {
        current = previous + 1;
        return;
    }

    // [+2,6] (2 bits)

    bool twoBits;
    if ( Stream::IsWriting )
        twoBits = difference <= 6;
    serialize_bool( stream, twoBits );
    if ( twoBits )
    {
        serialize_int( stream, difference, 2, 6 );
        if ( Stream::IsReading )
            current = previous + difference;
        return;
    }

    // [7,14] -> [0,7] (3 bits)

    bool threeBits;
    if ( Stream::IsWriting )
        threeBits = difference <= 14;
    serialize_bool( stream, threeBits );
    if ( threeBits )
    {
        serialize_int( stream, difference, 7, 14 );
        if ( Stream::IsReading )
            current = previous + difference;
        return;
    }

    // [15,30] -> [0,15] (4 bits)

    bool fourBits;
    if ( Stream::IsWriting )
        fourBits = difference <= 30;
    serialize_bool( stream, fourBits );
    if ( fourBits )
    {
        serialize_int( stream, difference, 15, 30 );
        if ( Stream::IsReading )
            current = previous + difference;
        return;
    }

    // [31,62] -> [0,31] (5 bits)

    bool fiveBits;
    if ( Stream::IsWriting )
        fiveBits = difference <= 62;
    serialize_bool( stream, fiveBits );
    if ( fiveBits )
    {
        serialize_int( stream, difference, 31, 62 );
        if ( Stream::IsReading )
            current = previous + difference;
        return;
    }

    // [63,126] -> [0,63] (6 bits)

    bool sixBits;
    if ( Stream::IsWriting )
        sixBits = difference <= 126;
    serialize_bool( stream, sixBits );
    if ( sixBits )
    {
        serialize_int( stream, difference, 63, 126 );
        if ( Stream::IsReading )
            current = previous + difference;
        return;
    }

    // [127,NumCubes]

    serialize_int( stream, difference, 127, NumCubes - 1 );
    if ( Stream::IsReading )
        current = previous + difference;
}

struct QuantizedCubeState
{
    bool interacting;

    int position_x;
    int position_y;
    int position_z;

    compressed_quaternion<OrientationBits> orientation;

    bool operator == ( const QuantizedCubeState & other ) const
    {
        if ( interacting != other.interacting )
            return false;

        if ( position_x != other.position_x )
            return false;

        if ( position_y != other.position_y )
            return false;

        if ( position_z != other.position_z )
            return false;

        if ( orientation != other.orientation )
            return false;

        return true;
    }

    bool operator != ( const QuantizedCubeState & other ) const
    {
        return ! ( *this == other );
    }
};

struct QuantizedSnapshot
{
    QuantizedCubeState cubes[NumCubes];

    bool operator == ( const QuantizedSnapshot & other ) const
    {
        for ( int i = 0; i < NumCubes; ++i )
        {
            if ( cubes[i] != other.cubes[i] )
                return false;
        }

        return true;
    }

    bool operator != ( const QuantizedSnapshot & other ) const
    {
        return ! ( *this == other );
    }
};

inline int signed_to_unsigned( int n )
{
    return ( n << 1 ) ^ ( n >> 31 );
}

inline int unsigned_to_signed( uint32_t n )
{
    return ( n >> 1 ) ^ ( -( n & 1 ) );
}

template <typename Stream> void serialize_unsigned_range( Stream & stream, uint32_t & value, int num_ranges, const int * range_bits )
{
    assert( num_ranges > 0 );

    int range_min = 0;
    
    for ( int i = 0; i < num_ranges - 1; ++i )
    {
        const int range_max = range_min + ( ( 1 << range_bits[i] ) - 1 );
        bool in_range = Stream::IsWriting && value <= range_max;
        serialize_bool( stream, in_range );
        if ( in_range )
        {
            serialize_int( stream, value, range_min, range_max );
            return;
        }
        range_min += ( 1 << range_bits[i] );
    }

    serialize_int( stream, value, range_min, range_min + ( ( 1 << range_bits[num_ranges-1] ) - 1 ) );
}

inline int unsigned_range_limit( int num_ranges, const int * range_bits )
{
    int range_limit = 0;
    for ( int i = 0; i < num_ranges; ++i )
        range_limit += ( 1 << range_bits[i] );
    return range_limit;
}

template <typename Stream> void serialize_relative_position( Stream & stream,
                                                             int & position_x,
                                                             int & position_y,
                                                             int & position_z,
                                                             int base_position_x,
                                                             int base_position_y,
                                                             int base_position_z )
{
    bool all_small;
    bool too_large;
    uint32_t dx,dy,dz;

    const int range_bits[] = { 5, 6, 7 };
    const int num_ranges = sizeof( range_bits ) / sizeof( int );

    const int small_limit = 15;
    const int large_limit = unsigned_range_limit( num_ranges, range_bits );

    const int max_delta = 2047;

    if ( Stream::IsWriting )
    {
        dx = signed_to_unsigned( position_x - base_position_x );
        dy = signed_to_unsigned( position_y - base_position_y );
        dz = signed_to_unsigned( position_z - base_position_z );
        all_small = dx <= small_limit && dy <= small_limit && dz <= small_limit;
        too_large = dx >= large_limit || dy >= large_limit || dz >= large_limit;
    }

    serialize_bool( stream, all_small );

    if ( all_small )
    {
        serialize_int( stream, dx, 0, small_limit );
        serialize_int( stream, dy, 0, small_limit );
        serialize_int( stream, dz, 0, small_limit );
    }
    else
    {
        serialize_bool( stream, too_large );

        if ( !too_large )
        {
            serialize_unsigned_range( stream, dx, num_ranges, range_bits );
            serialize_unsigned_range( stream, dy, num_ranges, range_bits );
            serialize_unsigned_range( stream, dz, num_ranges, range_bits );
        }
        else
        {
            serialize_int( stream, dx, 0, max_delta );
            serialize_int( stream, dy, 0, max_delta );            
            serialize_int( stream, dz, 0, max_delta );            
        }
    }

    if ( Stream::IsReading )
    {
        int signed_dx = unsigned_to_signed( dx );
        int signed_dy = unsigned_to_signed( dy );
        int signed_dz = unsigned_to_signed( dz );

        position_x = base_position_x + signed_dx;
        position_y = base_position_y + signed_dy;
        position_z = base_position_z + signed_dz;
    }
}

template <typename Stream> void serialize_relative_orientation( Stream & stream, 
                                                                compressed_quaternion<OrientationBits> & orientation, 
                                                                const compressed_quaternion<OrientationBits> & base_orientation )
{
    const int range_bits[] = { 4, 5, 7 };
    const int num_ranges = sizeof( range_bits ) / sizeof( int );

    const int small_limit = 3;
    const int large_limit = unsigned_range_limit( num_ranges, range_bits );

    uint32_t da,db,dc;
    bool all_small = false;
    bool relative_orientation = false;

    if ( Stream::IsWriting && orientation.largest == base_orientation.largest )
    {
        da = signed_to_unsigned( orientation.integer_a - base_orientation.integer_a );
        db = signed_to_unsigned( orientation.integer_b - base_orientation.integer_b );
        dc = signed_to_unsigned( orientation.integer_c - base_orientation.integer_c );

        all_small = da <= small_limit && db <= small_limit && dc <= small_limit;

        relative_orientation = da < large_limit && db < large_limit && dc < large_limit;
    }

    serialize_bool( stream, relative_orientation );

    if ( relative_orientation )
    {
        serialize_bool( stream, all_small );

        if ( all_small )
        {
            serialize_int( stream, da, 0, small_limit );
            serialize_int( stream, db, 0, small_limit );
            serialize_int( stream, dc, 0, small_limit );
        }
        else
        {
            serialize_unsigned_range( stream, da, num_ranges, range_bits );
            serialize_unsigned_range( stream, db, num_ranges, range_bits );
            serialize_unsigned_range( stream, dc, num_ranges, range_bits );
        }

        if ( Stream::IsReading )
        {
            int signed_da = unsigned_to_signed( da );
            int signed_db = unsigned_to_signed( db );
            int signed_dc = unsigned_to_signed( dc );

            orientation.largest = base_orientation.largest;
            orientation.integer_a = base_orientation.integer_a + signed_da;
            orientation.integer_b = base_orientation.integer_b + signed_db;
            orientation.integer_c = base_orientation.integer_c + signed_dc;
        }
    }
    else
    {
        serialize_object( stream, orientation );
    }
}

template <typename Stream> void serialize_cube_relative_to_base( Stream & stream, QuantizedCubeState & cube, const QuantizedCubeState & base, int base_dx, int base_dy, int base_dz )
{
    serialize_bool( stream, cube.interacting );

    bool position_changed;

    if ( Stream::IsWriting )
        position_changed = cube.position_x != base.position_x || cube.position_y != base.position_y || cube.position_z != base.position_z;

    serialize_bool( stream, position_changed );

    if ( position_changed )
    {
        const int gravity = 3;
        const int ground_limit = 105;

        const int drag_x = - ceil( base_dx * 0.062f );
        const int drag_y = - ceil( base_dy * 0.062f );
        const int drag_z = - ceil( base_dz * 0.062f );

        const int position_estimate_x = base.position_x + base_dx + drag_x;
        const int position_estimate_y = base.position_y + base_dy + drag_y;
        const int position_estimate_z = max( base.position_z + base_dz - gravity + drag_z, ground_limit );

        serialize_relative_position( stream, cube.position_x, cube.position_y, cube.position_z, position_estimate_x, position_estimate_y, position_estimate_z );
    }
    else if ( Stream::IsReading )
    {
        cube.position_x = base.position_x;
        cube.position_y = base.position_y;
        cube.position_z = base.position_z;
    }

    serialize_relative_orientation( stream, cube.orientation, base.orientation );
}

struct CompressionState
{
    float delta_x[NumCubes];
    float delta_y[NumCubes];
    float delta_z[NumCubes];
};

void calculate_compression_state( CompressionState & compression_state, QuantizedSnapshot & current_snapshot, QuantizedSnapshot & baseline_snapshot )
{
    for ( int i = 0; i < NumCubes; ++i )
    {
        compression_state.delta_x[i] = current_snapshot.cubes[i].position_x - baseline_snapshot.cubes[i].position_x;
        compression_state.delta_y[i] = current_snapshot.cubes[i].position_y - baseline_snapshot.cubes[i].position_y;
        compression_state.delta_z[i] = current_snapshot.cubes[i].position_z - baseline_snapshot.cubes[i].position_z;
    }
}

template <typename Stream> void serialize_snapshot_relative_to_baseline( Stream & stream, CompressionState & compression_state, QuantizedSnapshot & current_snapshot, QuantizedSnapshot & baseline_snapshot )
{
    QuantizedCubeState * quantized_cubes = &current_snapshot.cubes[0];
    QuantizedCubeState * quantized_base_cubes = &baseline_snapshot.cubes[0];

    const int MaxChanged = 256;

    int num_changed = 0;
    bool use_indices = false;
    bool changed[NumCubes];
    if ( Stream::IsWriting )
    {
        for ( int i = 0; i < NumCubes; ++i )
        {
            changed[i] = quantized_cubes[i] != quantized_base_cubes[i];
            if ( changed[i] )
                num_changed++;
        }

        if ( num_changed > 0 )
        {
            int relative_index_bits = count_relative_index_bits( changed );
            if ( num_changed <= MaxChanged && relative_index_bits <= NumCubes )
                use_indices = true;
        }
    }

    serialize_bool( stream, use_indices );

    if ( use_indices )
    {
        serialize_int( stream, num_changed, 1, MaxChanged );

        if ( Stream::IsWriting )
        {
            int num_written = 0;

            bool first = true;
            int previous_index = 0;

            for ( int i = 0; i < NumCubes; ++i )
            {
                if ( changed[i] )
                {
                    if ( first )
                    {
                        serialize_int( stream, i, 0, NumCubes - 1 );
                        first = false;
                    }
                    else
                    {   
                        serialize_relative_index( stream, previous_index, i );
                    }

                    serialize_cube_relative_to_base( stream, quantized_cubes[i], quantized_base_cubes[i], compression_state.delta_x[i], compression_state.delta_y[i], compression_state.delta_z[i] );

                    num_written++;

                    previous_index = i;
                }
            }

            assert( num_written == num_changed );
        }
        else
        {
            memset( changed, 0, sizeof( changed ) );

            int previous_index = 0;

            for ( int j = 0; j < num_changed; ++j )
            {
                int i;
                if ( j == 0 )
                    serialize_int( stream, i, 0, NumCubes - 1 );
                else                                
                    serialize_relative_index( stream, previous_index, i );

                serialize_cube_relative_to_base( stream, quantized_cubes[i], quantized_base_cubes[i], compression_state.delta_x[i], compression_state.delta_y[i], compression_state.delta_z[i] );

                changed[i] = true;

                previous_index = i;
            }

            for ( int i = 0; i < NumCubes; ++i )
            {
                if ( !changed[i] )
                    memcpy( &quantized_cubes[i], &quantized_base_cubes[i], sizeof( QuantizedCubeState ) );
            }
        }
    }
    else
    {
        for ( int i = 0; i < NumCubes; ++i )
        {
            serialize_bool( stream, changed[i] );

            if ( changed[i] )
            {
                serialize_cube_relative_to_base( stream, quantized_cubes[i], quantized_base_cubes[i], compression_state.delta_x[i], compression_state.delta_y[i], compression_state.delta_z[i] );
            }
            else if ( Stream::IsReading )
            {
                memcpy( &quantized_cubes[i], &quantized_base_cubes[i], sizeof( QuantizedCubeState ) );
            }
        }
    }
}

struct FrameCubeData
{
    int orientation_largest;
    int orientation_a;
    int orientation_b;
    int orientation_c;
    int position_x;
    int position_y;
    int position_z;
    int interacting;
};

struct Frame
{
    FrameCubeData cubes[NumCubes];
};

struct Packet
{
    int size;
    uint8_t data[MaxPacketSize];
};

void convert_frame_to_snapshot( const Frame & frame, QuantizedSnapshot & snapshot )
{
    for ( int j = 0; j < NumCubes; ++j )
    {
        assert( frame.cubes[j].orientation_largest >= 0 );
        assert( frame.cubes[j].orientation_largest <= 3 );

        snapshot.cubes[j].orientation.largest = frame.cubes[j].orientation_largest;

        assert( frame.cubes[j].orientation_a >= 0 );
        assert( frame.cubes[j].orientation_b >= 0 );
        assert( frame.cubes[j].orientation_c >= 0 );

        assert( frame.cubes[j].orientation_a <= ( 1 << OrientationBits ) - 1 );
        assert( frame.cubes[j].orientation_b <= ( 1 << OrientationBits ) - 1 );
        assert( frame.cubes[j].orientation_c <= ( 1 << OrientationBits ) - 1 );

        snapshot.cubes[j].orientation.integer_a = frame.cubes[j].orientation_a;
        snapshot.cubes[j].orientation.integer_b = frame.cubes[j].orientation_b;
        snapshot.cubes[j].orientation.integer_c = frame.cubes[j].orientation_c;

        assert( frame.cubes[j].position_x >= -QuantizedPositionBoundXY );
        assert( frame.cubes[j].position_y >= -QuantizedPositionBoundXY );
        assert( frame.cubes[j].position_z >= 0 );

        assert( frame.cubes[j].position_x <= QuantizedPositionBoundXY );
        assert( frame.cubes[j].position_y <= QuantizedPositionBoundXY );
        assert( frame.cubes[j].position_z <= QuantizedPositionBoundZ );

        snapshot.cubes[j].position_x = frame.cubes[j].position_x;
        snapshot.cubes[j].position_y = frame.cubes[j].position_y;
        snapshot.cubes[j].position_z = frame.cubes[j].position_z;

        assert( frame.cubes[j].interacting == 0 || frame.cubes[j].interacting == 1 );

        snapshot.cubes[j].interacting = frame.cubes[j].interacting;
    }
}

int main( int argc, char ** argv )
{
    FILE * file = fopen( "delta_data.bin", "rb" );
    if ( !file )
    {
        printf( "error: can't open file\n" );
        return 1;
    }

    // count number of frames in file

    fseek( file, 0L, SEEK_END );
    uint64_t file_size = ftell( file );
    fseek( file, 0L, SEEK_SET );

    const int num_frames = (int) floor( double(file_size) / sizeof( Frame ) );
    printf( "%d input frames\n", num_frames );
    assert( num_frames > 6 );

    // read in frames

    Frame * frames = new Frame[num_frames];
    uint64_t frames_read = fread( frames, sizeof( Frame ), num_frames, file );
    assert( frames_read == num_frames );
    fclose( file );

    // convert frames to snapshots

    QuantizedSnapshot * snapshots = new QuantizedSnapshot[num_frames];
    for ( int i = 0; i < num_frames; ++i )
        convert_frame_to_snapshot( frames[i], snapshots[i] );

    // write packets

    const int num_packets = num_frames - 6;
    printf( "writing %d packets\n", num_packets );
    assert( num_packets > 0 );

    CompressionState * compression_state = new CompressionState[num_frames];

    int packet_index = 0;
    Packet * packets = new Packet[num_packets];
    uint64_t total_bytes = 0;
    for ( int i = 6; i < num_frames; ++i )
    {
        Packet & packet = packets[packet_index];

        WriteStream stream( packet.data, MaxPacketSize );

        QuantizedSnapshot & current_snapshot = snapshots[i];
        QuantizedSnapshot & baseline_snapshot = snapshots[i-6];

        calculate_compression_state( compression_state[i], current_snapshot, baseline_snapshot );

        serialize_snapshot_relative_to_baseline( stream, compression_state[i-6], current_snapshot, baseline_snapshot );

        stream.Flush();

        int bits_written = stream.GetBitsProcessed();
        int bytes_written = ( bits_written / 8 ) + ( ( bits_written % 8 ) ? 1 : 0 );

        while ( packet.data[bytes_written] == 0 && bytes_written > 0 )
            --bytes_written;

        assert( bytes_written >= 0 );

        packet.size = bytes_written;
        total_bytes += bytes_written;

        ++packet_index;
    }

    // read packets and verify reconstruction of snapshot

    printf( "reading %d packets\n", num_packets );
    for ( int i = 0; i < num_packets; ++i )
    {
        Packet & packet = packets[i];

        ReadStream stream( packet.data, MaxPacketSize );

        QuantizedSnapshot current_snapshot;
        QuantizedSnapshot & baseline_snapshot = snapshots[i];

        serialize_snapshot_relative_to_baseline( stream, compression_state[i], current_snapshot, baseline_snapshot );

        assert( current_snapshot == snapshots[i+6] );
    }
    printf( "all packets verify ok!\n" );

    // print results

    printf( "total packet bytes: %llu\n", total_bytes );
    printf( "average bytes per-packet: %f\n", total_bytes / double(num_packets) );
    printf( "average bytes per-second: %f\n", total_bytes / double(num_packets) * 60 * 8 );
    printf( "average kilobits per-second: %f\n", total_bytes / double(num_packets) * 60 * 8 / 1000.0 );
    printf( "compression ratio: %.2f%% of original size\n", total_bytes / ( num_packets * ( 4 + 3 + 3 ) * 32 * NumCubes / 8.0 ) * 100.0 );

    // clean up everything

    delete [] snapshots;
    delete [] packets;
    delete [] frames;

    return 0;
}

/*
    2837 input frames
    writing 2831 packets
    reading 2831 packets
    all packets verify ok!
    total packet bytes: 1505956
    average bytes per-packet: 531.951960
    average bytes per-second: 255336.941010
    average kilobits per-second: 255.336941
    compression ratio: 1.48% of original size
*/
