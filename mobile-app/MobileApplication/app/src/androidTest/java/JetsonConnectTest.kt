import android.Manifest
import android.content.Context
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.filters.LargeTest
import androidx.test.rule.GrantPermissionRule
import com.example.testapp.BleManager
import com.example.testapp.model.SerializationDeserialization
import com.example.testapp.model.TickStreamPayload
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
@LargeTest
class JetsonConnectTest {
    private val serde = SerializationDeserialization

    @get:Rule
    val perms: GrantPermissionRule =
        GrantPermissionRule.grant(
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
        )

    @Test
    fun connectAndReceiveTick() =
        runBlocking {
            val ctx: Context = ApplicationProvider.getApplicationContext()
            val bleManager = BleManager(ctx, serde)

            try {
                withContext(Dispatchers.Main) {
                    bleManager.startScan { device ->
                        bleManager.connectToJetson(device)
                    }
                }

                val tick: TickStreamPayload =
                    try {
                        withTimeout(30_000) {
                            bleManager.ticks.first()
                        }
                    } catch (e: TimeoutCancellationException) {
                        throw AssertionError(
                            "Timed out waiting for a tick. Ensure Jetson is advertising ADAS service and sending notifications.",
                            e,
                        )
                    }

                assertNotNull(tick)
                assertTrue(tick.tickId >= 0)
            } finally {
                withContext(Dispatchers.Main) {
                    bleManager.disconnect()
                    bleManager.stopScan()
                }
            }
        }
}
